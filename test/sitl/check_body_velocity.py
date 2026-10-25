#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL body velocity test (ADR-0005, spec 0009): body velocity setpoints in TEST mode.

Starts SITL with its default ground vehicle (DIFF_2WD on outputs 1 and 2,
WHL_RADIUS 0.05 m, WHL_TRACK 0.2 m) and its UDP Link on a free port, and
drives it through the Link monitor library. In order:

- In STANDBY, body velocity setpoints are refused by setpoint arbitration
  (link_status counts them) and no output is driven.
- In TEST, armed, 0.3 m/s forward gives both wheels a 6 rad/s target and
  outputs 3 and 4 none (NaN), and the wheels' mean speed from 1 s on is
  within 5 % of it; 1 rad/s of yaw rate gives +2 rad/s left and -2 rad/s
  right.
- Switching levels: direct-actuator setpoints, then body velocity, then
  direct-actuator again. Every actuator_outputs sample shows the targets of
  exactly one level, never a mix, and the level sent last drives.
- Body velocity setpoints stopped, the vehicle disarms after COM_SP_TIMEOUT
  with every output zero, as for direct-actuator setpoints.
- The Link monitor's drive-body command drives the wheels by the
  kinematics and disarms at the end.

A source not in control cannot be shown here, as SITL has a single Link:
the Link work item's unit tests check it with two. The yaw rate loop's P
and I are set to 0 first, so the targets are the kinematics' exactly;
check_body_motion.py checks the loop.

    check_body_velocity.py build/test/app/vehkernel-sitl tools/link/link_monitor.py \\
        --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_actuators import (  # noqa: E402
    REACT_US, REASON, REPLY_S, SETPOINT_INTERVAL_S, SETTLED_S, SP_TIMEOUT_S, TEST, Vehicle, arm, set_param,
    stopped_after)
from vklink.session import ParamType, ParamValue  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402

RADIUS = 0.05
TRACK = 0.2
TOLERANCE = 0.05
DIRECT = (5.0, 5.0, 5.0, 5.0)
# The targets are judged from this long after a level's first setpoint left
# the host: what they are matters here, not how fast they appear, which
# check_actuators.py bounds for stops.
APPLIED_US = 100_000


def wheel_targets(vx: float, yaw_rate: float) -> tuple[float, float]:
    """The DIFF_2WD kinematics: the left and right wheel speeds [rad/s]."""
    return (vx + yaw_rate * TRACK / 2) / RADIUS, (vx - yaw_rate * TRACK / 2) / RADIUS


class BodyVehicle(Vehicle):
    """A Vehicle that also sends body velocity setpoints and records link_status."""

    def __init__(self, args, port: int):
        super().__init__(args, port)
        self.links: list = []
        self.monitor.listeners.append(self._record_link)

    def _record_link(self, name: str, instance: int, message) -> None:
        if name == "link_status" and instance == 0:
            self.links.append(message)

    def send_body(self, vx: float, vy: float, yaw_rate: float) -> float:
        setpoint = self.monitor.messages.BY_NAME["body_velocity_setpoint"]
        sent = time.monotonic()
        self.monitor.send_topic("body_velocity_setpoint", setpoint(vx=vx, vy=vy, yaw_rate=yaw_rate))
        return sent


def send_for(vehicle: BodyVehicle, send, duration_s: float) -> tuple[float, float]:
    """Calls `send` at 20 Hz for `duration_s`; the host times of the first and the last."""
    first = last = send()
    end = first + duration_s
    while (now := time.monotonic()) < end:
        if now - last >= SETPOINT_INTERVAL_S:
            last = send()
        vehicle.monitor.poll(min(last + SETPOINT_INTERVAL_S, end) - now)
    return first, last


def same(a: float, b: float) -> bool:
    return (math.isnan(a) and math.isnan(b)) or abs(a - b) < 1e-3


def targets_are(output, expected) -> bool:
    return all(same(t, e) for t, e in zip(output.target, expected))


def check_refused_in_standby(vehicle: BodyVehicle, failures: list[str]) -> None:
    vehicle.monitor.run(1.0)
    before = vehicle.links[-1].setpoints_rejected if vehicle.links else 0
    first, _ = send_for(vehicle, lambda: vehicle.send_body(0.3, 0.0, 0.0), 0.5)
    vehicle.monitor.run(1.2)  # link_status comes once a second
    rejected = (vehicle.links[-1].setpoints_rejected if vehicle.links else 0) - before
    if rejected < 5:
        failures.append(f"in STANDBY, only {rejected} body velocity setpoints were counted refused")
    if any(any(d != 0.0 for d in o.output) for o in vehicle.outputs if o.timestamp >= vehicle.mcu_us(first)):
        failures.append("in STANDBY, a body velocity setpoint drove an output")


def check_targets(vehicle: BodyVehicle, failures: list[str]) -> None:
    for vx, yaw_rate in ((0.3, 0.0), (0.0, 1.0)):
        left, right = wheel_targets(vx, yaw_rate)
        start = len(vehicle.outputs)
        first, _ = send_for(vehicle, lambda vx=vx, yaw_rate=yaw_rate: vehicle.send_body(vx, 0.0, yaw_rate), 1.5)
        late = [o for o in vehicle.outputs[start:] if o.timestamp >= vehicle.mcu_us(first) + APPLIED_US]
        expected = (left, right, math.nan, math.nan)
        wrong = [o for o in late if not targets_are(o, expected)]
        if not late or wrong:
            shown = ", ".join(f"{t:.2f}" for t in (wrong[0].target if wrong else ()))
            failures.append(f"vx {vx} yaw rate {yaw_rate}: targets {shown or 'none'}, expected "
                            f"{left:.2f}, {right:.2f}, nan, nan")
        # At 2 rad/s a wheel moves about 32 encoder counts a cycle, so single
        # samples jitter by a few percent: judge the mean of the steady part.
        reached = [e for e in vehicle.encoders if e.timestamp >= vehicle.mcu_us(first) + 1_000_000]
        mean = [sum(e.speed[i] for e in reached) / len(reached) for i in (0, 1)] if reached else []
        if not mean or abs(mean[0] - left) > abs(left) * TOLERANCE or abs(mean[1] - right) > abs(right) * TOLERANCE:
            speeds = ", ".join(f"{s:.2f}" for s in mean) or "none"
            failures.append(f"vx {vx} yaw rate {yaw_rate}: the wheels turned at {speeds} rad/s on average, "
                            f"not {left:.2f}, {right:.2f} within 5 %")


def check_levels(vehicle: BodyVehicle, failures: list[str]) -> None:
    body = (*wheel_targets(0.1, 0.0), math.nan, math.nan)
    start = len(vehicle.outputs)
    first, _ = send_for(vehicle, lambda: vehicle.send_setpoint(DIRECT), 0.6)
    switch_body, _ = send_for(vehicle, lambda: vehicle.send_body(0.1, 0.0, 0.0), 0.6)
    switch_direct, _ = send_for(vehicle, lambda: vehicle.send_setpoint(DIRECT), 0.6)
    vehicle.monitor.run(0.1)
    samples = [o for o in vehicle.outputs[start:] if o.timestamp >= vehicle.mcu_us(first) + APPLIED_US]
    mixed = [o for o in samples if not targets_are(o, DIRECT) and not targets_are(o, body)]
    if mixed:
        failures.append("switching levels, an output sample mixed them: "
                        + ", ".join(f"{t:.2f}" for t in mixed[0].target))

    def level_between(begin: float, end: float):
        window = [o for o in samples if vehicle.mcu_us(begin) + APPLIED_US <= o.timestamp <= vehicle.mcu_us(end)]
        return window and all(targets_are(o, DIRECT) for o in window), \
            window and all(targets_are(o, body) for o in window)

    if not level_between(first, switch_body)[0]:
        failures.append("the direct-actuator setpoints did not drive before body velocity was sent")
    if not level_between(switch_body, switch_direct)[1]:
        failures.append("body velocity, sent last, did not replace the direct-actuator setpoints")
    if not level_between(switch_direct, switch_direct + 0.6)[0]:
        failures.append("direct-actuator setpoints, sent last, did not replace body velocity")


def check_timeout(vehicle: BodyVehicle, failures: list[str]) -> None:
    _, last = send_for(vehicle, lambda: vehicle.send_body(0.2, 0.0, 0.0), 1.0)
    vehicle.monitor.run(SP_TIMEOUT_S + 0.5)
    problem = stopped_after(vehicle.outputs, vehicle.mcu_us(last) + int(SP_TIMEOUT_S * 1e6) + REACT_US)
    if problem:
        failures.append(f"body velocity stopped, the outputs were not zero after COM_SP_TIMEOUT: {problem}")
    status = vehicle.status()
    if status is None or status.armed or status.disarm_reason != REASON["setpoint timeout"]:
        failures.append("body velocity stopped, vehicle_status does not report a setpoint timeout disarm")


def check_drive_body_command(args, port: int, failures: list[str]) -> None:
    left, right = wheel_targets(0.2, 0.5)
    result = subprocess.run(
        [sys.executable, args.link_monitor, "--udp", f"127.0.0.1:{port}", "--msg-dir", args.msg_dir,
         "drive-body", "0.2,0,0.5", "--duration", "1.5"],
        capture_output=True, text=True, timeout=30, check=False)
    if result.returncode != 0:
        failures.append(f"drive-body failed: {result.stderr.strip()}")
        return
    # wheel_speed=<w1>,<w2>,<w3>,<w4> armed=0
    fields = dict(field.split("=", 1) for field in result.stdout.split())
    speeds = [float(s) for s in fields.get("wheel_speed", "nan,nan").split(",")]
    if fields.get("armed") != "0":
        failures.append(f"drive-body did not disarm at the end: {result.stdout.strip()}")
    if abs(speeds[0] - left) > abs(left) * 0.1 or abs(speeds[1] - right) > abs(right) * 0.1:
        failures.append(f"drive-body 0.2,0,0.5 turned the wheels at {speeds[0]:.2f}, {speeds[1]:.2f} rad/s, "
                        f"not {left:.2f}, {right:.2f}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("link_monitor")
    parser.add_argument("--msg-dir", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    port = free_udp_port()
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    vehicle = BodyVehicle(args, port)
    try:
        if not vehicle.connect():
            failures.append("no handshake with SITL")
        else:
            for name in ("RATE_YAW_P", "RATE_YAW_I"):
                if not isinstance(set_param(vehicle, name, ParamType.FLOAT, 0.0), ParamValue):
                    failures.append(f"{name} could not be set to 0")
            check_refused_in_standby(vehicle, failures)
            mode = vehicle.monitor.request_mode(TEST, REPLY_S)
            if mode is None or mode.refused:
                failures.append("mode TEST was not taken")
            vehicle.monitor.run(SETTLED_S)
            if arm(vehicle, failures, "in TEST"):
                check_targets(vehicle, failures)
                check_levels(vehicle, failures)
                check_timeout(vehicle, failures)
            vehicle.close()
            # The command's own connection takes over SITL's UDP Link.
            check_drive_body_command(args, port, failures)
    finally:
        vehicle.close()
        sitl.terminate()
        sitl.communicate(timeout=30)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: body velocity setpoints in TEST mode")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
