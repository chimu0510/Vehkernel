#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L2 SITL replay test (spec 0011, ADR-0018): records SITL driving a square, replays and judges."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools/log"))
sys.path.insert(0, str(REPO / "tools/link"))
sys.path.insert(0, str(REPO / "test/sitl"))

from compare import compare_logs, compute_odometry_metrics  # noqa: E402
from replay import find_replay_executable, replay  # noqa: E402
import vkulog  # noqa: E402
from vklink.recorder import ULogRecorder  # noqa: E402
from vklink.session import ParamType, ParamValue  # noqa: E402

from check_attitude import free_udp_port  # noqa: E402
from check_odometry import (  # noqa: E402
    HEADING_TOLERANCE_DEG,
    POSITION_FLOOR,
    POSITION_TOLERANCE,
    OdometryVehicle,
)
from check_remote_control import (  # noqa: E402
    SIDE_S,
    STICK,
    TURN_S,
    Gamepad,
    disarm,
    take_over_and_arm,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="path to vehkernel-sitl executable")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    parser.add_argument("--replay-bin", help="path to vehkernel-replay executable")
    args = parser.parse_args()

    failures: list[str] = []
    port = free_udp_port()
    sitl_cmd = [args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}"]
    sitl = subprocess.Popen(sitl_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    first_line = sitl.stdout.readline() if sitl.stdout else ""
    print(first_line, end="")

    vehicle = OdometryVehicle(args, port)
    try:
        if not vehicle.connect():
            failures.append("no handshake with SITL")
            return 1

        # Set a non-default parameter so that skipping parameters in replay causes a mismatch.
        reply = vehicle.monitor.set_param("ATT_KP", ParamType.FLOAT, 1.5, 3.0)
        if not isinstance(reply, ParamValue):
            failures.append(f"could not set ATT_KP: {reply}")
            return 1

        with tempfile.TemporaryDirectory() as tmp_dir:
            recorded_path = Path(tmp_dir) / "sitl_recorded.ulg"
            replayed_path = Path(tmp_dir) / "sitl_replayed.ulg"

            recorder = ULogRecorder(
                vehicle.monitor,
                "estimator_replay",
                recorded_path,
                msg_dir=args.msg_dir,
            )
            if not recorder.start():
                failures.append("failed to start ULogRecorder")
                return 1

            # Leave vehicle still for 1.5 s after log_start so the attitude estimator aligns
            vehicle.monitor.run(1.5)
            recorder.update()

            gamepad = Gamepad(vehicle)
            if not take_over_and_arm(vehicle, gamepad, failures, "square"):
                recorder.stop()
                return 1
            recorder.update()

            gamepad.hold(0.4)
            recorder.update()
            for _ in range(4):
                gamepad.hold(SIDE_S, sticks=(0.0, STICK, 0.0, 0.0))
                recorder.update()
                gamepad.hold(TURN_S, sticks=(0.0, 0.0, STICK, 0.0))
                recorder.update()

            gamepad.hold(0.5)
            recorder.update()
            disarm(gamepad)
            recorder.update()
            vehicle.monitor.run(0.5)
            recorder.update()

            recorder.stop()

            # Shut down SITL and vehicle monitor before replay
            vehicle.close()
            sitl.terminate()
            sitl.communicate(timeout=15)

            if not recorded_path.exists() or recorded_path.stat().st_size == 0:
                failures.append("recorded log file was not created or is empty")
                for f in failures:
                    print(f"FAIL: {f}", file=sys.stderr)
                return 1

            # 1. Check no dropouts in recorded log
            rec_reader = vkulog.ULogReader(recorded_path)
            if len(rec_reader.dropouts) != 0:
                failures.append(f"log recorded {len(rec_reader.dropouts)} dropouts; expected 0")
            print(f"Recorded log: {recorded_path.stat().st_size} bytes, {len(rec_reader.dropouts)} dropouts")

            # 2. Replay the log
            replay_exe = find_replay_executable(args.replay_bin)
            replay(recorded_path, replayed_path, executable=replay_exe)
            if not replayed_path.exists() or replayed_path.stat().st_size == 0:
                failures.append("replayed log file was not created or is empty")
                for f in failures:
                    print(f"FAIL: {f}", file=sys.stderr)
                return 1
            print(f"Replayed log: {replayed_path.stat().st_size} bytes")

            # 3. Compare logs for exact faithfulness (tolerance 0.0)
            faithful = compare_logs(recorded_path, replayed_path, tolerance=0.0)
            if not faithful:
                failures.append("replayed log is not faithful (mismatches detected with tolerance 0.0)")

            # 4. Check replayed odometry bounds against logged ground truth
            pos_error, heading_error_deg, driven = compute_odometry_metrics(recorded_path, replayed_path)
            max_pos_error = driven * POSITION_TOLERANCE + POSITION_FLOOR
            print(f"Odometry vs truth: driven {driven:.3f} m, position error {pos_error:.4f} m (bound {max_pos_error:.4f} m), "
                  f"heading error {heading_error_deg:.2f} deg (bound {HEADING_TOLERANCE_DEG:.1f} deg)")
            if pos_error > max_pos_error:
                failures.append(f"replayed odometry position error {pos_error:.4f} m exceeds bound {max_pos_error:.4f} m")
            if heading_error_deg > HEADING_TOLERANCE_DEG:
                failures.append(f"replayed odometry heading error {heading_error_deg:.2f} deg exceeds bound {HEADING_TOLERANCE_DEG:.1f} deg")

    finally:
        vehicle.close()
        if sitl.poll() is None:
            sitl.terminate()
            sitl.communicate(timeout=10)

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: sitl_replay: exact replay, odometry within bounds, no dropouts")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
