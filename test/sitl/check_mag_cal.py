#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL magnetometer calibration test (spec 0012, ADR-0018 L3):
calibrates hard-iron offsets and soft-iron scales from a turn-in-place log,
verifies recovery within 2 µT and 2 %, and confirms post-calibration heading error <= 3 deg.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mag_cal import calibrate_magnetometer, read_log_samples  # noqa: E402
from sitl_support import configure_persistent_params, free_udp_port  # noqa: E402
from vklink.recorder import ULogRecorder  # noqa: E402
from vklink.session import ParamType  # noqa: E402

from check_actuators import REPLY_S, SETTLED_S, TEST, arm  # noqa: E402
from check_odometry import OdometryVehicle  # noqa: E402
from check_vehicle import wrap, yaw_of  # noqa: E402

# Acceptance criteria tolerances specified in ticket #154 / spec 0012:
# - Recovered offsets within 2 µT of true simulated values
# - Recovered soft-iron scales within 2 % of expected values
# - Post-calibration EKF heading error within 3.0 degrees of ground truth
OFFSET_TOLERANCE_UT = 2.0
SCALE_TOLERANCE_REL = 0.02
HEADING_TOLERANCE_DEG = 3.0

# SITL simulated magnetic distortion parameters
SITL_MAG_OFFSET = (12.0, -8.0, 5.0)  # [µT]
SITL_MAG_SCALE = (1.08, 0.94, 1.00)

# Reasoning for expected calibration scales:
# In SITL, the body magnetic field is distorted by the soft-iron scales s_x = 1.08 and s_y = 0.94:
#   raw_x = s_x * body_x + offset_x
#   raw_y = s_y * body_y + offset_y
# An Earth horizontal field of radius R produces an axis-aligned ellipse with semi-axes:
#   a = s_x * R = 1.08 * R
#   b = s_y * R = 0.94 * R
# The 2D ellipse fit in mag_cal.py computes the mean radius r = (a + b) / 2 = R * (s_x + s_y) / 2.
# The calibration scale factors normalize each semi-axis to the mean radius r:
#   scale_x = r / a = ((s_x + s_y) / 2) / s_x = (1.08 + 0.94) / (2 * 1.08) = 1.01 / 1.08 ≈ 0.935185
#   scale_y = r / b = ((s_x + s_y) / 2) / s_y = (1.08 + 0.94) / (2 * 0.94) = 1.01 / 0.94 ≈ 1.074468
# These calibration scales are proportional to the reciprocals (1 / s_x, 1 / s_y) normalized to mean 1.
EXPECTED_SCALE_X = 1.01 / 1.08  # ≈ 0.935185
EXPECTED_SCALE_Y = 1.01 / 0.94  # ≈ 1.074468


class MagCalVehicle(OdometryVehicle):
    """An OdometryVehicle that also records vehicle_attitude and vehicle_attitude_groundtruth."""

    def __init__(self, args, port: int):
        super().__init__(args, port)
        self.attitude: list = []
        self.attitude_truth: list = []
        self.monitor.listeners.append(self._record_attitude)

    def _record_attitude(self, name: str, instance: int, msg: object) -> None:
        if name == "vehicle_attitude" and instance == 0:
            self.attitude.append(msg)
        elif name == "vehicle_attitude_groundtruth" and instance == 0:
            self.attitude_truth.append(msg)


def drive_and_check_heading(
    vehicle: MagCalVehicle,
    turn_twist: tuple[float, float, float] | None,
    duration_s: float,
    failures: list[str],
    label: str,
) -> None:
    """Executes an optional turn, allows settling, and checks EKF yaw against ground truth."""
    if turn_twist is not None and duration_s > 0.0:
        # The pause after the previous turn outlasts COM_SP_TIMEOUT, which disarmed the vehicle.
        if not arm(vehicle, failures, label):
            return
        yaw_before = yaw_of(vehicle.attitude_truth[-1].q) if vehicle.attitude_truth else None
        vehicle.monitor.drive_body(turn_twist, duration_s)
        vehicle.monitor.drive_body((0.0, 0.0, 0.0), 0.3)
        vehicle.monitor.run(1.0)
        if yaw_before is not None and vehicle.attitude_truth:
            turned = math.degrees(abs(wrap(yaw_of(vehicle.attitude_truth[-1].q) - yaw_before)))
            expected = math.degrees(abs(turn_twist[2]) * duration_s)
            # The wheel speed loops' start and stop leave the turn short of the setpoint's by a few degrees.
            if abs(turned - expected) > 20.0:
                failures.append(f"{label}: the vehicle turned {turned:.1f} deg, not about {expected:.0f} deg")
    else:
        vehicle.monitor.run(1.0)

    if not vehicle.attitude or not vehicle.attitude_truth:
        failures.append(f"{label}: missing attitude or truth samples")
        return

    latest_att = vehicle.attitude[-1]
    latest_truth = vehicle.attitude_truth[-1]

    est_yaw = yaw_of(latest_att.q)
    truth_yaw = yaw_of(latest_truth.q)
    err_deg = math.degrees(abs(wrap(est_yaw - truth_yaw)))

    print(
        f"{label}: est yaw {math.degrees(est_yaw):.1f} deg, truth yaw {math.degrees(truth_yaw):.1f} deg, "
        f"error {err_deg:.2f} deg (bound {HEADING_TOLERANCE_DEG:.1f} deg), mag_fused {latest_att.mag_fused}"
    )

    if latest_att.estimator_id != 2:
        failures.append(f"{label}: expected primary estimator_id 2 (EKF), got {latest_att.estimator_id}")
    if latest_att.mag_fused == 0:
        failures.append(f"{label}: expected non-zero mag_fused under heading fusion")
    if err_deg > HEADING_TOLERANCE_DEG:
        failures.append(f"{label}: heading error {err_deg:.2f} deg exceeds bound {HEADING_TOLERANCE_DEG:.1f} deg")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="path to vehkernel-sitl executable")
    parser.add_argument("--msg-dir", required=True, help="directory of generated vkmsg.py schema")
    args = parser.parse_args()

    failures: list[str] = []

    with tempfile.TemporaryDirectory() as tmp_dir:
        # ---------------------------------------------------------------------
        # Step 1: Record turn-in-place log with known hard/soft iron errors
        # ---------------------------------------------------------------------
        port1 = free_udp_port()
        mag_off_str = f"{SITL_MAG_OFFSET[0]},{SITL_MAG_OFFSET[1]},{SITL_MAG_OFFSET[2]}"
        mag_scale_str = f"{SITL_MAG_SCALE[0]},{SITL_MAG_SCALE[1]},{SITL_MAG_SCALE[2]}"

        sitl1_cmd = [
            args.sitl,
            "--duration", "0",
            "--udp", f"127.0.0.1:{port1}",
            "--mag-offset", mag_off_str,
            "--mag-scale", mag_scale_str,
        ]
        sitl1 = subprocess.Popen(sitl1_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(sitl1.stdout.readline(), end="")  # Link connection banner

        recorded_log = Path(tmp_dir) / "mag_cal_turn.ulg"
        vehicle1 = MagCalVehicle(args, port1)
        try:
            if not vehicle1.connect():
                failures.append("no handshake with SITL in run 1")
                return 1

            recorder = ULogRecorder(
                vehicle1.monitor,
                "estimator_replay",
                recorded_log,
                msg_dir=args.msg_dir,
            )
            if not recorder.start():
                failures.append("failed to start ULogRecorder")
                return 1

            # Leave still for 1.5 s after starting log so attitude estimator aligns
            vehicle1.monitor.run(1.5)
            recorder.update()

            mode_reply = vehicle1.monitor.request_mode(TEST, REPLY_S)
            if mode_reply is None or mode_reply.refused:
                failures.append("mode TEST was not taken in run 1")
                return 1
            vehicle1.monitor.run(SETTLED_S)
            recorder.update()

            if not arm(vehicle1, failures, "run 1 before turning"):
                return 1
            recorder.update()

            # Turn in place twice slowly (yaw rate 0.6 rad/s for 21.0 s => 12.6 rad ≈ 722 deg)
            print("Driving turn in place (2 full turns)...")
            vehicle1.monitor.drive_body((0.0, 0.0, 0.6), 21.0)
            recorder.update()

            vehicle1.monitor.drive_body((0.0, 0.0, 0.0), 0.5)
            vehicle1.monitor.run(0.5)
            recorder.update()

            recorder.stop()
        finally:
            vehicle1.close()
            sitl1.terminate()
            sitl1.communicate(timeout=10)

        if not recorded_log.exists() or recorded_log.stat().st_size == 0:
            failures.append("recorded calibration log was not created or is empty")
            for f in failures:
                print(f"FAIL: {f}", file=sys.stderr)
            return 1

        # ---------------------------------------------------------------------
        # Step 2: Calibrate from the log and verify recovered offsets and scales
        # ---------------------------------------------------------------------
        samples, cal_params = read_log_samples(recorded_log)
        result = calibrate_magnetometer(samples, cal_params, is_3d=False)

        x_off_err = abs(result.offsets[0] - SITL_MAG_OFFSET[0])
        y_off_err = abs(result.offsets[1] - SITL_MAG_OFFSET[1])
        x_scale_rel_err = abs(result.scales[0] - EXPECTED_SCALE_X) / EXPECTED_SCALE_X
        y_scale_rel_err = abs(result.scales[1] - EXPECTED_SCALE_Y) / EXPECTED_SCALE_Y

        print(
            f"Fitted X offset: {result.offsets[0]:.3f} µT (true: {SITL_MAG_OFFSET[0]:.1f} µT, "
            f"err {x_off_err:.3f} µT, bound {OFFSET_TOLERANCE_UT:.1f} µT)"
        )
        print(
            f"Fitted Y offset: {result.offsets[1]:.3f} µT (true: {SITL_MAG_OFFSET[1]:.1f} µT, "
            f"err {y_off_err:.3f} µT, bound {OFFSET_TOLERANCE_UT:.1f} µT)"
        )
        print(
            f"Fitted X scale:  {result.scales[0]:.4f} (expected: {EXPECTED_SCALE_X:.4f}, "
            f"err {x_scale_rel_err * 100:.2f}%, bound {SCALE_TOLERANCE_REL * 100:.1f}%)"
        )
        print(
            f"Fitted Y scale:  {result.scales[1]:.4f} (expected: {EXPECTED_SCALE_Y:.4f}, "
            f"err {y_scale_rel_err * 100:.2f}%, bound {SCALE_TOLERANCE_REL * 100:.1f}%)"
        )
        print(f"Coverage:        {result.coverage_deg:.1f} deg")
        print(f"Residual:        {result.residual_ut:.3f} µT ({result.residual_fraction * 100:.1f}% of r)")

        if x_off_err > OFFSET_TOLERANCE_UT:
            failures.append(f"X offset error {x_off_err:.3f} µT exceeds bound {OFFSET_TOLERANCE_UT:.1f} µT")
        if y_off_err > OFFSET_TOLERANCE_UT:
            failures.append(f"Y offset error {y_off_err:.3f} µT exceeds bound {OFFSET_TOLERANCE_UT:.1f} µT")
        if x_scale_rel_err > SCALE_TOLERANCE_REL:
            failures.append(f"X scale error {x_scale_rel_err * 100:.2f}% exceeds bound {SCALE_TOLERANCE_REL * 100:.1f}%")
        if y_scale_rel_err > SCALE_TOLERANCE_REL:
            failures.append(f"Y scale error {y_scale_rel_err * 100:.2f}% exceeds bound {SCALE_TOLERANCE_REL * 100:.1f}%")

        # ---------------------------------------------------------------------
        # Step 3: Apply parameters via mag_cal --apply and verify persistent heading
        # ---------------------------------------------------------------------
        param_file = str(Path(tmp_dir) / "persistent_params.bin")
        err = configure_persistent_params(
            args,
            param_file,
            [
                ("EST_PRIMARY", ParamType.INT32, 2),   # EKF
                ("EKF_MAG_TYPE", ParamType.INT32, 2),  # HEADING
            ],
        )
        if err is not None:
            failures.append(f"could not configure persistent params: {err}")
            return 1

        port2 = free_udp_port()
        sitl2_cmd = [
            args.sitl,
            "--duration", "0",
            "--udp", f"127.0.0.1:{port2}",
            "--mag-offset", mag_off_str,
            "--mag-scale", mag_scale_str,
            "--param-file", param_file,
        ]
        sitl2 = subprocess.Popen(sitl2_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(sitl2.stdout.readline(), end="")

        try:
            # Apply fitted calibration using mag_cal.py CLI over UDP Link
            apply_cmd = [
                sys.executable,
                str(REPO / "tools" / "log" / "mag_cal.py"),
                str(recorded_log),
                "--apply",
                "--udp", f"127.0.0.1:{port2}",
                "--msg-dir", args.msg_dir,
            ]
            apply_proc = subprocess.run(apply_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(apply_proc.stdout, end="")
            if apply_proc.returncode != 0:
                failures.append(f"mag_cal.py --apply failed (code {apply_proc.returncode}): {apply_proc.stderr}")
                return 1

            # Wait for parameter changes to be flushed to Flash (unsaved == 0)
            from vklink.monitor import LinkMonitor, UdpConnection, load_messages
            mon = LinkMonitor(UdpConnection("127.0.0.1", port2), load_messages(args.msg_dir))
            for _ in range(15):
                mon.poll(0.2)
                stats = mon.topics.get(("parameter_status", 0))
                if stats and stats.latest is not None and stats.latest.unsaved == 0:
                    break
            mon.connection.close()
        finally:
            sitl2.terminate()
            sitl2.communicate(timeout=10)

        # Start SITL run 3 with the same param_file to confirm persistence across restart
        port3 = free_udp_port()
        sitl3_cmd = [
            args.sitl,
            "--duration", "0",
            "--udp", f"127.0.0.1:{port3}",
            "--mag-offset", mag_off_str,
            "--mag-scale", mag_scale_str,
            "--param-file", param_file,
        ]
        sitl3 = subprocess.Popen(sitl3_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(sitl3.stdout.readline(), end="")

        vehicle3 = MagCalVehicle(args, port3)
        try:
            if not vehicle3.connect():
                failures.append("no handshake with SITL in run 3")
                return 1

            # Wait 2.0 s for boot and 1.0 s still alignment under calibrated magnetometer
            vehicle3.monitor.run(2.0)

            mode_reply = vehicle3.monitor.request_mode(TEST, REPLY_S)
            if mode_reply is None or mode_reply.refused:
                failures.append("mode TEST was not taken in run 3")
                return 1
            vehicle3.monitor.run(SETTLED_S)

            if not arm(vehicle3, failures, "run 3 before heading checks"):
                return 1

            # Turn to a sequence of headings and check heading error <= 3 deg
            drive_and_check_heading(vehicle3, None, 0.0, failures, "heading 0 deg")
            drive_and_check_heading(vehicle3, (0.0, 0.0, 0.5), math.pi, failures, "heading 90 deg")
            drive_and_check_heading(vehicle3, (0.0, 0.0, 0.5), math.pi, failures, "heading 180 deg")
            drive_and_check_heading(vehicle3, (0.0, 0.0, 0.5), math.pi, failures, "heading 270 deg")
            drive_and_check_heading(vehicle3, (0.0, 0.0, 0.5), math.pi, failures, "heading 360 deg")

        finally:
            vehicle3.close()
            sitl3.terminate()
            sitl3.communicate(timeout=10)

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: sitl_mag_cal: calibration recovered offsets and scales, heading error <= 3 deg")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
