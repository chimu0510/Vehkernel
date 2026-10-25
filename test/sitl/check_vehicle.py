#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL ground vehicle test (ADR-0018, spec 0009): the simulated motors move the vehicle.

Starts SITL with its default ground vehicle (DIFF_2WD on outputs 1 and 2,
the WHL_* defaults) and its UDP Link on a free port, drives the wheels by
direct-actuator setpoints through the Link monitor library, and compares
vehicle_local_odometry_groundtruth with what the wheels turned, read on
wheel_encoders:

- Forward at 10 rad/s on both wheels: the true distance is the wheels' mean
  turn times WHL_RADIUS, within 2 %, and the vehicle neither turns nor
  drifts sideways. The 2 % leaves room for the wheels' 16-bit encoder
  steps and the interpolation between truth samples.
- Turning in place, +4 and -4 rad/s: the true heading changes by
  WHL_RADIUS x (left - right) / WHL_TRACK, within 2 %, the position stays
  within 1 cm, and the attitude estimator's yaw, which sees the turn only
  through the simulated gyro, follows the true heading within 3 deg.
- With --slip 0.25: driving forward, the true distance is 75 % of the
  wheels' within 2 %.

Both topics arrive at 50 Hz but are sampled at different instants, so the
truth is interpolated to each wheel_encoders sample's time.

    check_vehicle.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_actuators import REPLY_S, SETTLED_S, TEST, Vehicle, arm, drive  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402

RADIUS = 0.05    # WHL_RADIUS's default [m]
TRACK = 0.2      # WHL_TRACK's default [m]
TOLERANCE = 0.02
FORWARD = (10.0, 10.0, math.nan, math.nan)
TURN = (4.0, -4.0, math.nan, math.nan)
SLIP = 0.25
# Let the wheels reach their speed and the vehicle settle before and after a window.
SETTLE_S = 0.6


def yaw_of(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(angle: float) -> float:
    return math.remainder(angle, 2 * math.pi)


class GroundVehicle(Vehicle):
    """A Vehicle that also records the ground truth and the attitude estimate."""

    def __init__(self, args, port: int):
        super().__init__(args, port)
        self.truth: list = []
        self.attitude: list = []
        self.monitor.listeners.append(self._record_more)

    def _record_more(self, name: str, _instance: int, message) -> None:
        if name == "vehicle_local_odometry_groundtruth":
            self.truth.append(message)
        elif name == "vehicle_attitude":
            self.attitude.append(message)


def nearest(samples, timestamp_us: int):
    return min(samples, key=lambda s: abs(s.timestamp - timestamp_us))


class Pose:
    """The truth's position and heading at one instant."""

    def __init__(self, position, yaw: float):
        self.position = position
        self.yaw = yaw


def truth_at(samples, timestamp_us: int) -> Pose:
    """The truth interpolated to `timestamp_us` between the samples around it."""
    before = max((s for s in samples if s.timestamp <= timestamp_us), key=lambda s: s.timestamp, default=None)
    after = min((s for s in samples if s.timestamp >= timestamp_us), key=lambda s: s.timestamp, default=None)
    if before is None or after is None or after.timestamp == before.timestamp:
        sample = nearest(samples, timestamp_us)
        return Pose(tuple(sample.position), yaw_of(sample.q))
    f = (timestamp_us - before.timestamp) / (after.timestamp - before.timestamp)
    position = tuple(a + (b - a) * f for a, b in zip(before.position, after.position))
    yaw = yaw_of(before.q) + wrap(yaw_of(after.q) - yaw_of(before.q)) * f
    return Pose(position, yaw)


def window(vehicle: GroundVehicle, speeds, duration_s: float):
    """Drives `speeds` for `duration_s`, then stops: the truth and the
    encoders at the start and the end of the steady part, after SETTLE_S."""
    first, last = drive(vehicle, speeds, duration_s)
    vehicle.monitor.run(0.2)
    start_us = vehicle.mcu_us(first + SETTLE_S)
    end_us = vehicle.mcu_us(last)
    if not vehicle.truth or not vehicle.encoders:
        return None
    e0, e1 = nearest(vehicle.encoders, start_us), nearest(vehicle.encoders, end_us)
    return truth_at(vehicle.truth, e0.timestamp), truth_at(vehicle.truth, e1.timestamp), e0, e1


def wheels_turned(start, end) -> tuple[float, float]:
    """How far the left and right wheels (outputs 1 and 2) turned [rad]."""
    return end.angle[0] - start.angle[0], end.angle[1] - start.angle[1]


def distance(start, end) -> float:
    return math.hypot(end.position[0] - start.position[0], end.position[1] - start.position[1])


def check_forward(vehicle: GroundVehicle, failures: list[str], slip: float = 0.0) -> None:
    what = f"forward with slip {slip}" if slip else "forward"
    w = window(vehicle, FORWARD, 2.0)
    if w is None:
        failures.append(f"{what}: no ground truth or wheel_encoders sample")
        return
    t0, t1, e0, e1 = w
    left, right = wheels_turned(e0, e1)
    expected = RADIUS * (left + right) / 2 * (1 - slip)
    moved = distance(t0, t1)
    if expected <= 0 or abs(moved - expected) > expected * TOLERANCE:
        failures.append(f"{what}: the vehicle moved {moved:.3f} m, its wheels say {expected:.3f} m")
    turned = math.degrees(abs(wrap(t1.yaw - t0.yaw)))
    if turned > 0.5:
        failures.append(f"{what}: the vehicle turned {turned:.2f} deg driving straight")
    sideways = max(abs(s.velocity[1]) for s in vehicle.truth)
    if sideways > 1e-3:
        failures.append(f"{what}: the vehicle moved sideways at {sideways:.3f} m/s")


def check_turn(vehicle: GroundVehicle, failures: list[str]) -> None:
    w = window(vehicle, TURN, 1.5)
    if w is None:
        failures.append("turning: no ground truth or wheel_encoders sample")
        return
    t0, t1, e0, e1 = w
    left, right = wheels_turned(e0, e1)
    expected = RADIUS * (left - right) / TRACK
    turned = wrap(t1.yaw - t0.yaw)
    # The heading wraps: compare the turn modulo a full circle.
    if abs(wrap(turned - expected)) > abs(expected) * TOLERANCE:
        failures.append(f"turning: the heading changed {math.degrees(turned):.1f} deg, "
                        f"the wheels say {math.degrees(wrap(expected)):.1f} deg")
    if distance(t0, t1) > 0.01:
        failures.append(f"turning in place, the vehicle moved {distance(t0, t1):.3f} m")
    # The estimate follows the truth through the gyro alone.
    worst = 0.0
    for estimate in vehicle.attitude:
        if estimate.timestamp_sample < e0.timestamp or estimate.timestamp_sample > e1.timestamp:
            continue
        truth = truth_at(vehicle.truth, estimate.timestamp_sample)
        worst = max(worst, abs(math.degrees(wrap(yaw_of(estimate.q) - truth.yaw))))
    if worst > 3.0 or not vehicle.attitude:
        failures.append(f"turning: the estimated yaw was up to {worst:.1f} deg from the true heading")


def run(args, slip: float, failures: list[str]) -> None:
    port = free_udp_port()
    command = [args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}"]
    if slip:
        command += ["--slip", str(slip)]
    sitl = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    vehicle = GroundVehicle(args, port)
    try:
        if not vehicle.connect():
            failures.append("no handshake with SITL")
            return
        # Wait for the board to be verified, as check_actuators does.
        vehicle.monitor.run(2.0)
        mode = vehicle.monitor.request_mode(TEST, REPLY_S)
        if mode is None or mode.refused:
            failures.append("mode TEST was not taken")
            return
        vehicle.monitor.run(SETTLED_S)
        if not arm(vehicle, failures, "before driving"):
            return
        if slip:
            check_forward(vehicle, failures, slip)
        else:
            check_forward(vehicle, failures)
            check_turn(vehicle, failures)
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
        print("PASS: the simulated motors move the ground vehicle as its wheel kinematics say")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
