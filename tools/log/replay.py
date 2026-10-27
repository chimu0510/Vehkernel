#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Feeds vehkernel-replay from a log, and writes its output as a log (spec 0011, ADR-0018 L2)."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import threading
from typing import Any

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))

import vkulog  # noqa: E402
from vkulog.reader import DataMessage, ParameterChange  # noqa: E402


# Parameters a board's definition adds (the baud rate of each serial port). A board log holds them, the
# replay program, built for the host, does not know them, and no estimator reads them: they are set aside
# without --allow-unknown-parameters, while any other unknown name still fails the replay.
BOARD_ONLY_PARAMETER = re.compile(r"SERIAL\d+_BAUD")


def find_replay_executable(explicit: str | Path | None = None) -> Path:
    if explicit:
        p = Path(explicit)
        if p.is_file() and os.access(p, os.X_OK):
            return p
        raise FileNotFoundError(f"vehkernel-replay executable not found at {explicit}")

    env_path = os.environ.get("VEHKERNEL_REPLAY")
    if env_path:
        p = Path(env_path)
        if p.is_file() and os.access(p, os.X_OK):
            return p

    candidates = [
        REPO / "build" / "test" / "test" / "vehkernel-replay",
        REPO / "build" / "test" / "vehkernel-replay",
        REPO / "build" / "sitl" / "test" / "vehkernel-replay",
        REPO / "build" / "sitl" / "vehkernel-replay",
    ]
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return c

    which_path = shutil.which("vehkernel-replay")
    if which_path:
        return Path(which_path)

    raise FileNotFoundError("vehkernel-replay executable not found; build the test preset first")


def read_exact(stream, n: int) -> bytes:
    data = bytearray()
    while len(data) < n:
        chunk = stream.read(n - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)


def replay(
    in_path: str | Path,
    out_path: str | Path,
    executable: str | Path | None = None,
    allow_unknown_parameters: bool = False,
) -> None:
    exe = find_replay_executable(executable)
    reader = vkulog.ULogReader(in_path)

    topics_by_msg_id = {sub.msg_id: sub.message_name for sub in reader.subscriptions.values()}
    instances_by_msg_id = {sub.msg_id: sub.multi_id for sub in reader.subscriptions.values()}

    # Gather in-flight records in timestamp and priority order:
    # Priority 0: Parameter change
    # Priority 1: estimator_restart
    # Priority 2: imu_sample
    # Priority 3: body_velocity_measurement
    in_flight: list[tuple[int, int, str, Any]] = []

    for r in reader.records:
        if isinstance(r, ParameterChange):
            in_flight.append((r.timestamp_us, 0, "P", (r.name, r.value)))
        elif isinstance(r, DataMessage):
            name = topics_by_msg_id.get(r.msg_id)
            inst = instances_by_msg_id.get(r.msg_id, 0)
            if name == "estimator_restart":
                in_flight.append((r.timestamp_us, 1, "D", (name, inst, r.data)))
            elif name == "imu_sample" and inst == 0:
                in_flight.append((r.timestamp_us, 2, "D", (name, inst, r.data)))
            elif name == "body_velocity_measurement" and inst == 0:
                in_flight.append((r.timestamp_us, 3, "D", (name, inst, r.data)))

    in_flight.sort(key=lambda item: (item[0], item[1]))

    proc = subprocess.Popen(
        [str(exe)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    output_records: list[tuple[str, int, bytes]] = []
    reader_error: list[Exception] = []

    def stdout_worker() -> None:
        try:
            assert proc.stdout is not None
            while True:
                magic_bytes = read_exact(proc.stdout, 1)
                if not magic_bytes:
                    break
                magic = magic_bytes[0]
                if magic != ord("D"):
                    raise ValueError(f"Unknown record type from vehkernel-replay: {magic!r}")
                nlen = read_exact(proc.stdout, 1)[0]
                name = read_exact(proc.stdout, nlen).decode("ascii")
                inst = read_exact(proc.stdout, 1)[0]
                (plen,) = struct.unpack("<H", read_exact(proc.stdout, 2))
                payload = read_exact(proc.stdout, plen)
                output_records.append((name, inst, payload))
        except Exception as e:  # noqa: BLE001
            reader_error.append(e)

    reader_thread = threading.Thread(target=stdout_worker)
    reader_thread.start()

    assert proc.stdin is not None
    # 1. Feed initial parameters with timestamp = 0
    for name, val in reader.parameters.items():
        name_bytes = name.encode("ascii")
        if isinstance(val, float):
            ptype = 2
            vbits = struct.pack("<f", val)
        else:
            ptype = 1
            vbits = struct.pack("<i", int(val))
        pkt = struct.pack("<BQBB", ord("P"), 0, ptype, len(name_bytes)) + name_bytes + vbits
        proc.stdin.write(pkt)

    # 2. Feed in-flight records
    for ts, _, kind, data in in_flight:
        if kind == "P":
            name, val = data
            name_bytes = name.encode("ascii")
            if isinstance(val, float):
                ptype = 2
                vbits = struct.pack("<f", val)
            else:
                ptype = 1
                vbits = struct.pack("<i", int(val))
            pkt = struct.pack("<BQBB", ord("P"), ts, ptype, len(name_bytes)) + name_bytes + vbits
            proc.stdin.write(pkt)
        elif kind == "D":
            name, inst, payload = data
            name_bytes = name.encode("ascii")
            pkt = struct.pack("<BBB", ord("D"), len(name_bytes), inst) + name_bytes + struct.pack("<H", len(payload)) + payload
            # Wait, notice format in test/replay/main.cpp:
            # magic (1 byte 'D')
            # nlen (1 byte)
            # name (nlen bytes)
            # instance (1 byte)
            # plen (2 bytes)
            # payload (plen bytes)
            pkt = struct.pack("<BB", ord("D"), len(name_bytes)) + name_bytes + struct.pack("<BH", inst, len(payload)) + payload
            proc.stdin.write(pkt)

    proc.stdin.close()
    reader_thread.join()
    stderr_data = proc.stderr.read() if proc.stderr else b""
    proc.wait()

    stderr_text = stderr_data.decode(errors="replace")
    unknown_params: list[str] = []
    for line in stderr_text.splitlines():
        if line.startswith("unknown parameter:"):
            param_name = line.partition(":")[2].strip()
            if param_name and param_name not in unknown_params:
                unknown_params.append(param_name)

    board_only = [name for name in unknown_params if BOARD_ONLY_PARAMETER.fullmatch(name)]
    unknown_params = [name for name in unknown_params if name not in board_only]
    if board_only:
        print(f"Board-only parameters in log, ignored: {', '.join(board_only)}", file=sys.stderr)
    if unknown_params:
        print(f"Unknown parameters in log: {len(unknown_params)} ({', '.join(unknown_params)})", file=sys.stderr)
        if not allow_unknown_parameters:
            raise RuntimeError(
                f"Replay log contains {len(unknown_params)} unknown parameters: {', '.join(unknown_params)} "
                "(use --allow-unknown-parameters to proceed)"
            )

    if reader_error:
        raise reader_error[0]
    if proc.returncode != 0:
        raise RuntimeError(f"vehkernel-replay exited with code {proc.returncode}: {stderr_text}")

    # Write output log
    with vkulog.ULogWriter(out_path, start_timestamp_us=reader.start_timestamp_us) as writer:
        # Formats
        writer.write_format(vkulog.FORMATS["vehicle_attitude"])
        writer.write_format(vkulog.FORMATS["vehicle_local_odometry"])

        # Info messages
        for key, val in reader.info.items():
            writer.write_info(key, val)

        # Initial parameters
        for name, val in reader.parameters.items():
            writer.write_parameter(name, val)

        # Add logged messages
        writer.write_add_logged(msg_id=1, message_name="vehicle_attitude", multi_id=0)
        writer.write_add_logged(msg_id=2, message_name="vehicle_local_odometry", multi_id=0)

        # Merge output data and in-flight parameter changes in timestamp order.
        # Parameter changes in ULog have their timestamp inferred from the preceding
        # data message, so write them after data messages of the same timestamp.
        merged_events: list[tuple[int, int, Any]] = []
        for change in reader.parameter_changes:
            merged_events.append((change.timestamp_us, 2, change))

        for name, inst, payload in output_records:
            (ts,) = struct.unpack("<Q", payload[:8])
            msg_id = 1 if name == "vehicle_attitude" else 2
            merged_events.append((ts, 1, (msg_id, payload)))

        merged_events.sort(key=lambda ev: (ev[0], ev[1]))

        for _, kind, ev_data in merged_events:
            if kind == 2:
                change = ev_data
                writer.write_parameter(change.name, change.value)
            elif kind == 1:
                msg_id, payload = ev_data
                writer.write_data(msg_id, payload)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("in_log", help="recorded ULog file to replay")
    parser.add_argument("out_log", help="path for replayed ULog file")
    parser.add_argument("--executable", help="path to vehkernel-replay executable")
    parser.add_argument(
        "--allow-unknown-parameters",
        action="store_true",
        help="allow parameters not known to vehkernel-replay",
    )
    args = parser.parse_args()

    try:
        replay(
            args.in_log,
            args.out_log,
            args.executable,
            allow_unknown_parameters=args.allow_unknown_parameters,
        )
    except RuntimeError as err:
        print(f"ERROR: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
