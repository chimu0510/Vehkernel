#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL log recording test (spec 0011): records a log over the Link and validates it."""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/log"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools/link"))

import pyulog
import vkulog

MARGIN_S = 4.0


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("monitor", help="tools/link/link_monitor.py")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    parser.add_argument("--duration", type=float, default=30.0, help="seconds to record")
    args = parser.parse_args()

    port = free_udp_port()
    address = f"127.0.0.1:{port}"
    sitl_duration = args.duration + 2 * MARGIN_S
    sitl_args = [args.sitl, "--duration", str(sitl_duration), "--udp", address]

    sitl = subprocess.Popen(sitl_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    first_line = sitl.stdout.readline()
    print(first_line, end="")

    with tempfile.TemporaryDirectory() as tmp_dir:
        log_path = Path(tmp_dir) / "sitl_recorded.ulg"
        monitor_args = [
            sys.executable, args.monitor, "--udp", address,
            "--msg-dir", args.msg_dir, "--handshake-timeout", str(MARGIN_S),
            "record", str(log_path), "--profile", "estimator_replay",
            "--duration", str(args.duration),
        ]

        monitor = subprocess.run(monitor_args, capture_output=True, text=True,
                                 timeout=args.duration + 30, check=False)
        sitl_stdout, sitl_stderr = sitl.communicate(timeout=30)

        print(monitor.stdout, end="")
        print(monitor.stderr, end="", file=sys.stderr)
        print(sitl_stdout, end="")

        failures = []
        if monitor.returncode != 0:
            failures.append(f"link_monitor record failed with exit code {monitor.returncode}")
        if sitl.returncode != 0:
            failures.append(f"vehkernel-sitl exited with {sitl.returncode}: {sitl_stderr}")

        if not log_path.exists() or log_path.stat().st_size == 0:
            failures.append("recorded log file was not created or is empty")
            for f in failures:
                print(f"FAIL: {f}", file=sys.stderr)
            return 1

        # 1. Open with pyulog
        try:
            ulog = pyulog.ULog(str(log_path))
        except Exception as error:  # noqa: BLE001
            failures.append(f"pyulog failed to open recorded log: {error}")
            for f in failures:
                print(f"FAIL: {f}", file=sys.stderr)
            return 1

        # 2. Check no dropouts with vkulog.ULogReader
        reader = vkulog.ULogReader(log_path)
        if len(reader.dropouts) != 0:
            failures.append(f"log recorded {len(reader.dropouts)} dropouts; expected 0")

        # 3. Import profile definition
        sys.path.insert(0, args.msg_dir)
        try:
            from vklog_profiles import PROFILES  # type: ignore
        except ImportError as error:
            failures.append(f"could not load vklog_profiles from {args.msg_dir}: {error}")
            for f in failures:
                print(f"FAIL: {f}", file=sys.stderr)
            return 1

        profile = PROFILES["estimator_replay"]
        data_by_topic: dict[tuple[str, int], pyulog.core.ULog.Data] = {}
        for d in ulog.data_list:
            data_by_topic[(d.name, d.multi_id)] = d

        sitl_publications: dict[tuple[str, int], int] = {}
        for line in sitl_stdout.splitlines():
            if not line.startswith("topic "):
                continue
            pairs = dict(pair.split("=", 1) for pair in line.split()[1:] if "=" in pair)
            if "name" in pairs and "instance" in pairs and "publications" in pairs:
                sitl_publications[(pairs["name"], int(pairs["instance"]))] = int(pairs["publications"])

        print(f"\n--- Measured topic rates over {args.duration:g} s ---")
        for entry in profile.entries:
            if entry.kconfig == "PLATFORM_SITL" or entry.kconfig is None:
                expected_hz = entry.rate_hz
                if expected_hz <= 0.0:
                    continue  # as_published without nominal rate, e.g. parameter_update

                instances = [entry.instance] if entry.instance is not None else list(range(entry.instance_count))
                for inst in instances:
                    # Unused work item slots or inactive links are never published by SITL
                    # (publications == 0 in SITL exit report). Skip rate check and do not fail.
                    if sitl_publications and sitl_publications.get((entry.topic, inst), 0) == 0:
                        print(f"  {entry.topic}:{inst:<2} skipped (never published in SITL)")
                        continue

                    dataset = data_by_topic.get((entry.topic, inst))
                    if dataset is None:
                        failures.append(f"profile topic {entry.topic}:{inst} missing in log")
                        continue

                    count = len(dataset.data["timestamp"])
                    measured_hz = count / args.duration
                    rel_err = abs(measured_hz - expected_hz) / expected_hz
                    print(f"  {entry.topic}:{inst:<2} expected={expected_hz:6.1f} Hz  "
                          f"measured={measured_hz:6.2f} Hz  error={rel_err*100.0:4.1f}% ({count} msgs)")

                    if rel_err > 0.05:
                        failures.append(
                            f"topic {entry.topic}:{inst} rate {measured_hz:.2f} Hz exceeds 5% tolerance "
                            f"from expected {expected_hz:.1f} Hz (error {rel_err*100.0:.1f}%)"
                        )

        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)

        return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
