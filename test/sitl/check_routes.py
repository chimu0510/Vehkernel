#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL route test (ADR-0018, spec 0009): body velocity routes against the truth.

Starts SITL with its default ground vehicle (DIFF_2WD) and its UDP Link on
a free port, and drives two routes with the Link monitor library's
drive_body, the drive-body command's own call, judging
vehicle_local_odometry_groundtruth and vehicle_local_odometry:

- Forward 1 m: 0.25 m/s for 4 s. The true distance is within 5 % of 1 m:
  velocity control is feedforward to the wheel speed loops, whose wheels
  take about 0.1 s to reach their speed and to stop, and the setpoints'
  timing over the Link is the host's; together a few centimetres either
  way. The odometry's distance agrees with the true one within 3 %, and
  the vehicle keeps its heading within 2°.
- A square: four sides of 0.25 m/s for 2 s and four right turns in place at
  1 rad/s for pi/2 s. The odometry ends within 5 cm and 3° of the truth: the
  turns' small errors swing every later side, and dead reckoning follows
  them with the attitude's heading, so the two must end together whatever
  the square's own closure. The truth must also have turned about a full
  circle, within 15°, which shows the turns were made.

    check_routes.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_attitude import free_udp_port  # noqa: E402
from check_body_motion import connect_and_arm  # noqa: E402
from check_odometry import OdometryVehicle  # noqa: E402
from check_vehicle import truth_at, wrap, yaw_of  # noqa: E402

FORWARD = (0.25, 0.0, 0.0)
FORWARD_S = 4.0
DISTANCE_TOLERANCE = 0.05
ODOMETRY_TOLERANCE = 0.03
STRAIGHT_HEADING_DEG = 2.0
SIDE_S = 2.0
TURN = (0.0, 0.0, 1.0)
SQUARE_POSITION_M = 0.05
SQUARE_HEADING_DEG = 3.0
SQUARE_TURN_DEG = 15.0


def drive_body(vehicle: OdometryVehicle, twist, duration_s: float) -> None:
    """One leg with the monitor's drive_body, as the drive-body command drives."""
    vehicle.monitor.drive_body(twist, duration_s)


def settle(vehicle: OdometryVehicle) -> None:
    """Stops and lets the wheels and the samples settle."""
    drive_body(vehicle, (0.0, 0.0, 0.0), 0.4)
    vehicle.monitor.run(0.2)


def latest_pair(vehicle: OdometryVehicle):
    """The newest odometry sample and the truth at its time."""
    odometry = vehicle.odometry[-1]
    return odometry, truth_at(vehicle.truth, odometry.timestamp_sample)


def check_forward(vehicle: OdometryVehicle, failures: list[str]) -> None:
    settle(vehicle)
    if not vehicle.odometry or not vehicle.truth:
        failures.append("forward: no odometry or ground truth")
        return
    o0, t0 = latest_pair(vehicle)
    drive_body(vehicle, FORWARD, FORWARD_S)
    settle(vehicle)
    o1, t1 = latest_pair(vehicle)
    true = math.hypot(t1.position[0] - t0.position[0], t1.position[1] - t0.position[1])
    odometry = math.hypot(o1.position[0] - o0.position[0], o1.position[1] - o0.position[1])
    turned = math.degrees(abs(wrap(t1.yaw - t0.yaw)))
    print(f"forward: true {true:.4f} m, odometry {odometry:.4f} m, turned {turned:.2f} deg")
    if abs(true - 1.0) > DISTANCE_TOLERANCE:
        failures.append(f"forward 1 m: the vehicle moved {true:.3f} m")
    if abs(odometry - true) > true * ODOMETRY_TOLERANCE:
        failures.append(f"forward 1 m: the odometry says {odometry:.3f} m, the truth {true:.3f} m")
    if turned > STRAIGHT_HEADING_DEG:
        failures.append(f"forward 1 m: the vehicle turned {turned:.1f} deg")


def check_square(vehicle: OdometryVehicle, failures: list[str]) -> None:
    settle(vehicle)
    o0, t0 = latest_pair(vehicle)
    start = len(vehicle.truth)
    for _ in range(4):
        drive_body(vehicle, (0.25, 0.0, 0.0), SIDE_S)
        drive_body(vehicle, TURN, math.pi / 2)
    settle(vehicle)
    o1, t1 = latest_pair(vehicle)
    truth = vehicle.truth[start:]
    turned = math.degrees(sum(wrap(yaw_of(b.q) - yaw_of(a.q)) for a, b in zip(truth, truth[1:])))
    error = math.hypot((o1.position[0] - o0.position[0]) - (t1.position[0] - t0.position[0]),
                       (o1.position[1] - o0.position[1]) - (t1.position[1] - t0.position[1]))
    heading = math.degrees(abs(wrap(yaw_of(o1.q) - t1.yaw)))
    closure = math.hypot(t1.position[0] - t0.position[0], t1.position[1] - t0.position[1])
    print(f"square: true turn {turned:.1f} deg, true closure {closure:.3f} m, "
          f"odometry error {error:.4f} m, heading error {heading:.2f} deg")
    if abs(turned - 360.0) > SQUARE_TURN_DEG:
        failures.append(f"square: the vehicle turned {turned:.0f} deg in all, not a full circle")
    if error > SQUARE_POSITION_M:
        failures.append(f"square: the odometry ended {error:.3f} m from the truth")
    if heading > SQUARE_HEADING_DEG:
        failures.append(f"square: the odometry's heading ended {heading:.1f} deg from the truth")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("--msg-dir", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    port = free_udp_port()
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    vehicle = OdometryVehicle(args, port)
    try:
        if connect_and_arm(vehicle, failures):
            check_forward(vehicle, failures)
            check_square(vehicle, failures)
    finally:
        vehicle.close()
        sitl.terminate()
        sitl.communicate(timeout=30)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: forward 1 m and a square, against the truth")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
