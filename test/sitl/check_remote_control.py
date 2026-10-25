#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL remote control test (spec 0010): the simulated gamepad drives the vehicle in MANUAL.

Starts SITL with its default ground vehicle and its UDP Link on a free port,
and plays the gamepad with sim_gamepad frames through the Link monitor
library, judging vehicle_status and vehicle_local_odometry_groundtruth:

- A takes over in MANUAL; START held 1.2 s arms; the left stick at 0.54
  forward (half of MAN_VX_MAX after the deadzone, 0.25 m/s) moves the
  vehicle forward, 0.5 m in 2 s within 15 %: the stick's speed with the
  wheels' 0.1 s lag at either end and the host's frame timing; BACK held
  1.2 s disarms with REQUEST.
- Armed again and driving, the gamepad unplugged: the vehicle disarms with
  RC_LOST, within the gamepad's last report and the next control cycles.
- Arming with a stick off centre is refused as STICKS_NOT_CENTRED.

    check_remote_control.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_actuators import REASON, SETTLED_S  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402
from check_body_motion import MotionVehicle  # noqa: E402
from vklink.enums import ArmingResult, GamepadButton, Mode  # noqa: E402

HOLD_S = 1.2
FORWARD = 0.54        # stick travel: (0.54 - 0.08) / 0.92 = 0.5 of MAN_VX_MAX
DRIVE_S = 2.0
EXPECTED_M = 0.25 * DRIVE_S
DISTANCE_TOLERANCE = 0.15
GAMEPAD_INTERVAL_S = 0.05


def button(*names: str) -> int:
    return sum(1 << GamepadButton[name] for name in names)


class Gamepad:
    """The simulated gamepad, set again every 50 ms so the Link stays alive while it is held."""

    def __init__(self, vehicle: MotionVehicle):
        self.vehicle = vehicle

    def hold(self, duration_s: float, sticks=(0.0, 0.0, 0.0, 0.0), buttons: int = 0) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.vehicle.monitor.sim_gamepad(True, sticks, buttons=buttons)
            self.vehicle.monitor.run(min(GAMEPAD_INTERVAL_S, max(0.0, end - time.monotonic())))
        self.vehicle.monitor.sim_gamepad(True)

    def unplug(self) -> None:
        self.vehicle.monitor.sim_gamepad(False)


def latest_status(vehicle: MotionVehicle):
    vehicle.monitor.run(0.3)
    return vehicle.statuses[-1] if vehicle.statuses else None


def check_drive(vehicle: MotionVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    gamepad.hold(0.3)
    gamepad.hold(0.2, buttons=button("A"))
    gamepad.hold(0.2)
    status = latest_status(vehicle)
    if status is None or status.mode != Mode.MANUAL or status.source != 0x10:
        failures.append("A did not take over in MANUAL with the remote control in control")
        return
    gamepad.hold(HOLD_S, buttons=button("START"))
    status = latest_status(vehicle)
    if status is None or not status.armed:
        refusal = ArmingResult(status.arming_refusal).name if status else "no status"
        failures.append(f"START held {HOLD_S} s did not arm: {refusal}")
        return
    start = vehicle.truth[-1] if vehicle.truth else None
    gamepad.hold(DRIVE_S, sticks=(0.0, FORWARD, 0.0, 0.0))
    gamepad.hold(0.5)
    end = vehicle.truth[-1] if vehicle.truth else None
    if start is None or end is None:
        failures.append("no ground truth while driving")
    else:
        moved = math.hypot(end.position[0] - start.position[0], end.position[1] - start.position[1])
        print(f"drive: left stick {FORWARD} for {DRIVE_S} s moved {moved:.3f} m, {EXPECTED_M:.2f} m expected")
        if abs(moved - EXPECTED_M) > EXPECTED_M * DISTANCE_TOLERANCE or end.position[0] < start.position[0]:
            failures.append(f"the left stick moved the vehicle {moved:.3f} m, not {EXPECTED_M:.2f} m forward")
    gamepad.hold(HOLD_S, buttons=button("BACK"))
    status = latest_status(vehicle)
    if status is None or status.armed or status.disarm_reason != REASON["request"]:
        failures.append("BACK held did not disarm with REQUEST")


def check_sticks_off_centre(vehicle: MotionVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    gamepad.hold(HOLD_S, sticks=(0.0, 0.0, 0.5, 0.0), buttons=button("START"))
    status = latest_status(vehicle)
    if status is None or status.armed or status.arming_refusal != ArmingResult.STICKS_NOT_CENTRED:
        failures.append("arming with the right stick off centre was not refused as STICKS_NOT_CENTRED")


def check_unplugged(vehicle: MotionVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    gamepad.hold(HOLD_S, buttons=button("START"))
    gamepad.hold(1.0, sticks=(0.0, FORWARD, 0.0, 0.0))
    statuses = len(vehicle.statuses)
    gamepad.unplug()
    vehicle.monitor.run(1.0)
    disarm = next((s for s in vehicle.statuses[statuses:] if not s.armed), None)
    if disarm is None or disarm.disarm_reason != REASON["rc lost"]:
        failures.append("the gamepad unplugged while driving did not disarm with RC_LOST")
    elif vehicle.outputs and any(any(d != 0.0 for d in o.output) for o in vehicle.outputs[-10:]):
        failures.append("the gamepad unplugged, the outputs did not stop")


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
    vehicle = MotionVehicle(args, port)
    try:
        if not vehicle.connect():
            failures.append("no handshake with SITL")
        else:
            vehicle.monitor.run(2.0 + SETTLED_S)
            gamepad = Gamepad(vehicle)
            check_drive(vehicle, gamepad, failures)
            check_sticks_off_centre(vehicle, gamepad, failures)
            check_unplugged(vehicle, gamepad, failures)
    finally:
        vehicle.close()
        sitl.terminate()
        sitl.communicate(timeout=30)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: the simulated gamepad arms, drives and disarms; unplugged, the vehicle stops with RC_LOST")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
