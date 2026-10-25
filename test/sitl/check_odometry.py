#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL odometry test (ADR-0014, spec 0009): local odometry against the truth.

Starts SITL with its default ground vehicle (DIFF_2WD) and its UDP Link on
a free port, drives it by body velocity setpoints through the Link monitor
library, and compares vehicle_local_odometry and body_velocity_measurement
with vehicle_local_odometry_groundtruth:

- Without slip: 0.3 m/s forward for 2 s, a right turn in place at 1 rad/s
  for about 90°, and 0.3 m/s forward again. The odometry's position ends
  within 3 % of the distance driven plus 2 cm of the truth, and its heading
  within 3°. The measurement's speed matches the truth's within 2 % while
  driving straight.
- With --slip 0.25, turning in place at 1 rad/s for 3 s: the odometry's
  heading, which is the attitude estimator's, stays within 3° of the true
  heading, while the heading the wheels' speed difference gives (the
  measurement's yaw rate integrated) runs a third ahead of it, as the wheels
  turn 1 / 0.75 as far as the vehicle does.

Both topics arrive at 50 Hz but are sampled at different instants, so the
truth is interpolated to each odometry sample's time.

    check_odometry.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_attitude import free_udp_port  # noqa: E402
from check_body_motion import MotionVehicle, connect_and_arm  # noqa: E402
from check_body_velocity import send_for  # noqa: E402
from check_vehicle import truth_at, wrap, yaw_of  # noqa: E402

SLIP = 0.25
POSITION_TOLERANCE = 0.03   # of the distance driven
POSITION_FLOOR = 0.02       # [m]
HEADING_TOLERANCE_DEG = 3.0
SPEED_TOLERANCE = 0.02


class OdometryVehicle(MotionVehicle):
    """A MotionVehicle that also records the odometry and the measurement."""

    def __init__(self, args, port: int):
        super().__init__(args, port)
        self.odometry: list = []
        self.measurement: list = []
        self.monitor.listeners.append(self._record_odometry)

    def _record_odometry(self, name: str, instance: int, message) -> None:
        if name == "vehicle_local_odometry":
            self.odometry.append(message)
        elif name == "body_velocity_measurement" and instance == 0:
            self.measurement.append(message)


def drive(vehicle: OdometryVehicle, twist, duration_s: float) -> tuple[float, float]:
    return send_for(vehicle, lambda: vehicle.send_body(*twist), duration_s)


def check_route(vehicle: OdometryVehicle, failures: list[str]) -> None:
    first, _ = drive(vehicle, (0.3, 0.0, 0.0), 2.0)
    straight_from, straight_to = vehicle.mcu_us(first + 0.8), vehicle.mcu_us(first + 2.0)
    drive(vehicle, (0.0, 0.0, 1.0), math.pi / 2)
    drive(vehicle, (0.3, 0.0, 0.0), 2.0)
    drive(vehicle, (0.0, 0.0, 0.0), 0.5)
    vehicle.monitor.run(0.2)
    if not vehicle.odometry or not vehicle.truth or not vehicle.measurement:
        failures.append("no odometry, measurement or ground truth")
        return

    # Driving straight, the measured speed is the true speed.
    measured = [m.vx for m in vehicle.measurement if straight_from <= m.timestamp_sample <= straight_to]
    true = [t.velocity[0] for t in vehicle.truth if straight_from <= t.timestamp <= straight_to]
    if not measured or not true:
        failures.append("no measurement or ground truth while driving straight")
    else:
        m, t = sum(measured) / len(measured), sum(true) / len(true)
        print(f"straight: measured vx {m:.4f} m/s, true vx {t:.4f} m/s")
        if abs(m - t) > t * SPEED_TOLERANCE:
            failures.append(f"driving straight, the measured speed {m:.3f} m/s was not the true {t:.3f} m/s")

    end = vehicle.odometry[-1]
    truth = truth_at(vehicle.truth, end.timestamp_sample)
    driven = sum(math.hypot(b.position[0] - a.position[0], b.position[1] - a.position[1])
                 for a, b in zip(vehicle.truth, vehicle.truth[1:]))
    error = math.hypot(end.position[0] - truth.position[0], end.position[1] - truth.position[1])
    heading = math.degrees(abs(wrap(yaw_of(end.q) - truth.yaw)))
    print(f"route: driven {driven:.3f} m, odometry ({end.position[0]:.3f}, {end.position[1]:.3f}), truth "
          f"({truth.position[0]:.3f}, {truth.position[1]:.3f}), error {error:.3f} m, heading error {heading:.2f} deg")
    if error > driven * POSITION_TOLERANCE + POSITION_FLOOR:
        failures.append(f"the odometry ended {error:.3f} m from the truth after {driven:.2f} m")
    if heading > HEADING_TOLERANCE_DEG:
        failures.append(f"the odometry's heading ended {heading:.1f} deg from the truth")


def check_slip_heading(vehicle: OdometryVehicle, failures: list[str]) -> None:
    first, last = drive(vehicle, (0.0, 0.0, 1.0), 3.0)
    drive(vehicle, (0.0, 0.0, 0.0), 0.5)
    vehicle.monitor.run(0.2)
    start_us, end_us = vehicle.mcu_us(first), vehicle.mcu_us(last + 0.5)
    odometry = [o for o in vehicle.odometry if start_us <= o.timestamp_sample <= end_us]
    measured = [m for m in vehicle.measurement if start_us <= m.timestamp_sample <= end_us]
    if len(odometry) < 2 or len(measured) < 2:
        failures.append("slip: no odometry or measurement while turning")
        return
    # The turn is over half a circle: add up the wrapped steps between samples.
    truth = [t for t in vehicle.truth if odometry[0].timestamp_sample <= t.timestamp <= odometry[-1].timestamp_sample]
    true_turn = sum(wrap(yaw_of(b.q) - yaw_of(a.q)) for a, b in zip(truth, truth[1:]))
    worst = max(abs(math.degrees(wrap(yaw_of(o.q) - truth_at(vehicle.truth, o.timestamp_sample).yaw)))
                for o in odometry)
    wheels = sum(b.yaw_rate * (b.timestamp_sample - a.timestamp_sample) * 1e-6 for a, b in zip(measured, measured[1:]))
    print(f"slip: true turn {math.degrees(true_turn):.1f} deg, the wheels' {math.degrees(wheels):.1f} deg, "
          f"odometry heading error up to {worst:.2f} deg")
    if worst > HEADING_TOLERANCE_DEG:
        failures.append(f"with slip, the odometry's heading was up to {worst:.1f} deg from the truth")
    if not wheels > true_turn * 1.2:
        failures.append(f"with slip, the wheels' heading turned {math.degrees(wheels):.0f} deg against the true "
                        f"{math.degrees(true_turn):.0f} deg: slip did not show")


def run(args, slip: float, failures: list[str]) -> None:
    port = free_udp_port()
    command = [args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}"]
    if slip:
        command += ["--slip", str(slip)]
    sitl = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    vehicle = OdometryVehicle(args, port)
    try:
        if connect_and_arm(vehicle, failures):
            if slip:
                check_slip_heading(vehicle, failures)
            else:
                check_route(vehicle, failures)
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
        print("PASS: local odometry follows the truth, with the attitude's heading")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
