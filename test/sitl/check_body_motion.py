#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL body motion test (ADR-0013, spec 0009): the control chain against the true motion.

Starts SITL with its default ground vehicle (DIFF_2WD, WHL_RADIUS 0.05 m,
WHL_TRACK 0.2 m) and its UDP Link on a free port, drives it by body velocity
setpoints through the Link monitor library, and judges
vehicle_local_odometry_groundtruth and allocation_status:

- A circle, 0.3 m/s at 0.5 rad/s for 8 s, without slip and with --slip 0.25:
  from 4 s on, the true yaw rate's mean is within 3 % of 0.5 rad/s and no
  sample is more than 10 % off. With slip the wheels alone would turn the
  vehicle 25 % too slowly; only the yaw rate loop on the gyro closes that
  gap, so this checks it. The 4 s let the loop's integral settle, and the
  10 % covers the gyro's noise in single samples.
- A twist beyond the wheels' reach, 0.4 m/s at 2 rad/s with MOT_MAX_SPD at
  10 rad/s (the wheels would need 12 and 4): allocation_status reports vx
  and the yaw rate limited with a scale near 5/6, and the true path keeps
  the commanded curvature, yaw rate over speed 5 rad/m, within 5 %.
- Stopping from the circle, without slip and with it (#133): the setpoint
  steps to zero, and the true speed falls no faster than VEL_DEC_MAX
  (1 m/s²) allows, taking at least 0.9 × 0.3 s. Once stopped the heading
  holds within 1.5°: with slip the yaw rate loop's integral has grown to
  make up for it, and held over a stop it turned the vehicle 4.5° in 2.8 s
  and kept turning; at rest it is held at zero, and the heading settles
  within about 0.6°.

    check_body_motion.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_actuators import REPLY_S, SETTLED_S, TEST, arm, set_param  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402
from check_body_velocity import BodyVehicle, send_for  # noqa: E402
from vklink.session import ParamType, ParamValue  # noqa: E402

CIRCLE = (0.3, 0.0, 0.5)
SLIP = 0.25
SETTLE_S = 4.0
MEAN_TOLERANCE = 0.03
SAMPLE_TOLERANCE = 0.10
SATURATED = (0.4, 0.0, 2.0)
SATURATED_MAX_SPD = 10.0
CURVATURE_TOLERANCE = 0.05
VX, YAW_RATE = 1, 4
DEC_MAX = 1.0             # VEL_DEC_MAX's default
STOPPED_SPEED = 0.01      # m/s
STOP_HOLD_S = 2.0         # judged from the stop on
HEADING_TOLERANCE = 1.5   # degrees


class MotionVehicle(BodyVehicle):
    """A BodyVehicle that also records the ground truth and allocation_status."""

    def __init__(self, args, port: int):
        super().__init__(args, port)
        self.truth: list = []
        self.allocation: list = []
        self.monitor.listeners.append(self._record_motion)

    def _record_motion(self, name: str, _instance: int, message) -> None:
        if name == "vehicle_local_odometry_groundtruth":
            self.truth.append(message)
        elif name == "allocation_status":
            self.allocation.append(message)


def drive(vehicle: MotionVehicle, twist, duration_s: float, settle_s: float):
    """Drives `twist` for `duration_s`; the truth and allocation_status samples after `settle_s`."""
    truth, allocation = len(vehicle.truth), len(vehicle.allocation)
    first, last = send_for(vehicle, lambda: vehicle.send_body(*twist), duration_s)
    vehicle.monitor.run(0.1)
    start, end = vehicle.mcu_us(first + settle_s), vehicle.mcu_us(last)
    return ([t for t in vehicle.truth[truth:] if start <= t.timestamp <= end],
            [a for a in vehicle.allocation[allocation:] if start <= a.timestamp <= end])


def connect_and_arm(vehicle: MotionVehicle, failures: list[str]) -> bool:
    if not vehicle.connect():
        failures.append("no handshake with SITL")
        return False
    vehicle.monitor.run(2.0)
    mode = vehicle.monitor.request_mode(TEST, REPLY_S)
    if mode is None or mode.refused:
        failures.append("mode TEST was not taken")
        return False
    vehicle.monitor.run(SETTLED_S)
    return arm(vehicle, failures, "before driving")


def check_circle(vehicle: MotionVehicle, slip: float, failures: list[str]) -> None:
    what = f"circle with slip {slip}" if slip else "circle"
    truth, _ = drive(vehicle, CIRCLE, 8.0, SETTLE_S)
    if not truth:
        failures.append(f"{what}: no ground truth")
        return
    target = CIRCLE[2]
    rates = [t.angular_velocity[2] for t in truth]
    mean = sum(rates) / len(rates)
    worst = max(rates, key=lambda r: abs(r - target))
    print(f"{what}: true yaw rate mean {mean:.4f} rad/s, worst sample {worst:.4f} rad/s")
    if abs(mean - target) > target * MEAN_TOLERANCE:
        failures.append(f"{what}: the true yaw rate averaged {mean:.3f} rad/s, not {target} within 3 %")
    if abs(worst - target) > target * SAMPLE_TOLERANCE:
        failures.append(f"{what}: a true yaw rate sample was {worst:.3f} rad/s, more than 10 % off {target}")


def heading_deg(t) -> float:
    w, x, y, z = t.q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def check_stop(vehicle: MotionVehicle, slip: float, failures: list[str]) -> None:
    what = f"stop with slip {slip}" if slip else "stop"
    start = len(vehicle.truth)
    first, _ = send_for(vehicle, lambda: vehicle.send_body(0.0, 0.0, 0.0), STOP_HOLD_S + 1.0)
    truth = [t for t in vehicle.truth[start:] if t.timestamp >= vehicle.mcu_us(first)]
    stopped = next((t for t in truth if abs(t.velocity[0]) < STOPPED_SPEED), None)
    if stopped is None:
        failures.append(f"{what}: the vehicle did not stop")
        return
    took_s = (stopped.timestamp - vehicle.mcu_us(first)) / 1e6
    shortest_s = 0.9 * CIRCLE[0] / DEC_MAX
    held = [heading_deg(t) for t in truth if t.timestamp >= stopped.timestamp + 200_000]
    drift = max(held) - min(held) if held else 0.0
    print(f"{what}: stopped after {took_s:.3f} s, heading drifted {drift:.2f} degrees")
    if took_s < shortest_s:
        failures.append(f"{what}: the vehicle stopped in {took_s:.3f} s, faster than VEL_DEC_MAX allows "
                        f"({shortest_s:.2f} s)")
    if drift > HEADING_TOLERANCE:
        failures.append(f"{what}: the heading drifted {drift:.2f} degrees after stopping, more than "
                        f"{HEADING_TOLERANCE}")


def check_saturated(vehicle: MotionVehicle, failures: list[str]) -> None:
    if not isinstance(set_param(vehicle, "MOT_MAX_SPD", ParamType.FLOAT, SATURATED_MAX_SPD), ParamValue):
        failures.append("MOT_MAX_SPD could not be set")
        return
    truth, allocation = drive(vehicle, SATURATED, 4.0, 2.0)
    if not truth or not allocation:
        failures.append("saturated: no ground truth or allocation_status")
        return
    limited = [a for a in allocation if (a.limited & (VX | YAW_RATE)) != (VX | YAW_RATE)]
    if limited:
        failures.append(f"saturated: allocation_status reported limited={limited[0].limited}, not vx and yaw rate")
    scale = sum(a.scale for a in allocation) / len(allocation)
    if abs(scale - 10.0 / 12.0) > 0.05:
        failures.append(f"saturated: the allocation scaled by {scale:.3f}, not about 5/6")
    vx = sum(t.velocity[0] for t in truth) / len(truth)
    yaw_rate = sum(t.angular_velocity[2] for t in truth) / len(truth)
    curvature = yaw_rate / vx if vx else float("inf")
    expected = SATURATED[2] / SATURATED[0]
    print(f"saturated: scale {scale:.3f}, true vx {vx:.3f} m/s, yaw rate {yaw_rate:.3f} rad/s, "
          f"curvature {curvature:.3f} rad/m")
    if abs(curvature - expected) > expected * CURVATURE_TOLERANCE:
        failures.append(f"saturated: the true curvature was {curvature:.2f} rad/m, not {expected:.2f} within 5 %")


def run(args, slip: float, failures: list[str]) -> None:
    port = free_udp_port()
    command = [args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}"]
    if slip:
        command += ["--slip", str(slip)]
    sitl = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    vehicle = MotionVehicle(args, port)
    try:
        if connect_and_arm(vehicle, failures):
            check_circle(vehicle, slip, failures)
            check_stop(vehicle, slip, failures)
            if not slip:
                check_saturated(vehicle, failures)
    finally:
        vehicle.close()
        sitl.terminate()
        sitl.communicate(timeout=30)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("--msg-dir", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    run(args, 0.0, failures)
    run(args, SLIP, failures)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: the control chain holds the yaw rate and the curvature")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
