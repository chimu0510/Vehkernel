#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL remote control test (ADR-0018, spec 0010): the simulated gamepad against the truth.

Starts SITL with its default ground vehicle and its UDP Link on a free port,
and plays the gamepad with sim_gamepad frames through the Link monitor
library, judging vehicle_status, actuator_outputs,
vehicle_local_odometry_groundtruth and vehicle_local_odometry. In order:

- Arming refused: in MANUAL with the gamepad unplugged, a Link cannot arm
  on its behalf (NOT_IN_CONTROL); with the right stick off centre, START
  held is refused as STICKS_NOT_CENTRED. RC_NOT_CONNECTED cannot be asked
  for by a pad that is gone; the commander's unit tests check it.
- A takes over in MANUAL; START held 1.2 s arms; a square with the sticks
  (the left stick at 0.54, 0.25 m/s, for 2 s; the right stick at 0.54,
  0.5 rad/s, for pi s, four times), then BACK held 1.2 s disarms with
  REQUEST. The truth must have turned a full circle within 20°: the turns
  are timed by the host's frames, about 2° each way; the vehicle must end
  within 0.15 m of where it started, as the stick-driven sides and turns
  close the square within those timings; and the odometry must end within
  5 cm and 3° of the truth, as in check_routes.py.
- Takeover: the host arms in TEST and drives at 0.2 m/s; A switches to
  MANUAL without disarming; the host's setpoints, still arriving, are then
  dropped and counted on vehicle_status, and the vehicle stops within 1 s
  with the sticks centred; the left stick then drives it again.
- RC failsafe: unplugged while driving, the vehicle disarms with RC_LOST
  and every output is zero; connected but silent, it disarms with RC_LOST
  between RC_TIMEOUT (0.5 s) less 50 ms and RC_TIMEOUT + 0.3 s after the
  host silenced it: the 50 ms for the last report, which left up to one
  20 ms report interval before the host's request, and the 0.3 s for
  vehicle_status's 5 Hz and the host's timing.
- The gamepad's emergency stop: B zeroes every output within 0.1 s; Y held
  1.2 s releases it and the sticks drive again; B held off for 5 s disarms
  with EMERGENCY_STOP.

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

from check_actuators import REASON, REPLY_S, SETTLED_S, TEST, arm  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402
from check_body_velocity import send_for  # noqa: E402
from check_odometry import OdometryVehicle  # noqa: E402
from check_vehicle import truth_at, wrap, yaw_of  # noqa: E402
from vklink.enums import ArmingResult, GamepadButton, Mode  # noqa: E402

HOLD_S = 1.2
STICK = 0.54          # (0.54 - 0.08) / 0.92 = half of MAN_VX_MAX or MAN_YAW_MAX
SIDE_S = 2.0
TURN_S = math.pi      # 0.5 rad/s for pi s: 90°
SQUARE_TURN_DEG = 20.0
SQUARE_CLOSURE_M = 0.15
ODOMETRY_M = 0.05
ODOMETRY_DEG = 3.0
STOP_S = 1.0
STOPPED_MS = 0.02
RC_TIMEOUT_S = 0.5
RC_LOST_MARGIN_S = 0.3
ESTOP_REACT_S = 0.1
ESTOP_DISARM_S = 5.0
GAMEPAD_INTERVAL_S = 0.05


def button(*names: str) -> int:
    return sum(1 << GamepadButton[name] for name in names)


class Gamepad:
    """The simulated gamepad, set again every 50 ms so the Link stays alive while it is held."""

    def __init__(self, vehicle: OdometryVehicle):
        self.vehicle = vehicle

    def hold(self, duration_s: float, sticks=(0.0, 0.0, 0.0, 0.0), buttons: int = 0) -> None:
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            self.vehicle.monitor.sim_gamepad(True, sticks, buttons=buttons)
            self.vehicle.monitor.run(min(GAMEPAD_INTERVAL_S, max(0.0, end - time.monotonic())))
        self.vehicle.monitor.sim_gamepad(True)

    def press(self, *names: str) -> None:
        self.hold(0.2, buttons=button(*names))
        self.hold(0.2)

    def unplug(self) -> None:
        self.vehicle.monitor.sim_gamepad(False)

    def silence(self) -> None:
        self.vehicle.monitor.sim_gamepad(True, silent=True)


def latest_status(vehicle: OdometryVehicle):
    vehicle.monitor.run(0.3)
    return vehicle.statuses[-1] if vehicle.statuses else None


def refusal(status) -> str:
    return ArmingResult(status.arming_refusal).name if status else "no status"


def take_over_and_arm(vehicle: OdometryVehicle, gamepad: Gamepad, failures: list[str], what: str) -> bool:
    gamepad.press("A")
    status = latest_status(vehicle)
    if status is None or status.mode != Mode.MANUAL or status.source != 0x10:
        failures.append(f"{what}: A did not take over in MANUAL with the remote control in control")
        return False
    if not status.armed:
        gamepad.hold(HOLD_S, buttons=button("START"))
        status = latest_status(vehicle)
    if status is None or not status.armed:
        failures.append(f"{what}: START held {HOLD_S} s did not arm: {refusal(status)}")
        return False
    return True


def disarm(gamepad: Gamepad) -> None:
    gamepad.hold(HOLD_S, buttons=button("BACK"))


def check_arming_refused(vehicle: OdometryVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    gamepad.press("A")
    gamepad.unplug()
    vehicle.monitor.run(RC_TIMEOUT_S + 0.2)
    # The vehicle is in MANUAL with the remote control in control, but it is gone.
    reply = vehicle.monitor.request_arming(True, REPLY_S)
    status = latest_status(vehicle)
    if status is None or status.arming_refusal != ArmingResult.NOT_IN_CONTROL or (reply and not reply.refused):
        failures.append(f"a Link could arm in MANUAL: {refusal(status)}")
    gamepad.hold(0.3)
    gamepad.hold(HOLD_S, sticks=(0.0, 0.0, 0.5, 0.0), buttons=button("START"))
    status = latest_status(vehicle)
    if status is None or status.armed or status.arming_refusal != ArmingResult.STICKS_NOT_CENTRED:
        failures.append(f"arming with the right stick off centre was not refused as STICKS_NOT_CENTRED: "
                        f"{refusal(status)}")


def check_square(vehicle: OdometryVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    if not take_over_and_arm(vehicle, gamepad, failures, "square"):
        return
    gamepad.hold(0.4)
    o0 = vehicle.odometry[-1] if vehicle.odometry else None
    start = len(vehicle.truth)
    for _ in range(4):
        gamepad.hold(SIDE_S, sticks=(0.0, STICK, 0.0, 0.0))
        gamepad.hold(TURN_S, sticks=(0.0, 0.0, STICK, 0.0))
    gamepad.hold(0.5)
    disarm(gamepad)
    status = latest_status(vehicle)
    if status is None or status.armed or status.disarm_reason != REASON["request"]:
        failures.append("BACK held did not disarm with REQUEST")
    truth = vehicle.truth[start:]
    o1 = vehicle.odometry[-1] if vehicle.odometry else None
    if len(truth) < 2 or o0 is None or o1 is None:
        failures.append("square: no ground truth or odometry")
        return
    turned = math.degrees(sum(wrap(yaw_of(b.q) - yaw_of(a.q)) for a, b in zip(truth, truth[1:])))
    closure = math.hypot(truth[-1].position[0] - truth[0].position[0], truth[-1].position[1] - truth[0].position[1])
    t0, t1 = truth_at(vehicle.truth, o0.timestamp_sample), truth_at(vehicle.truth, o1.timestamp_sample)
    error = math.hypot((o1.position[0] - o0.position[0]) - (t1.position[0] - t0.position[0]),
                       (o1.position[1] - o0.position[1]) - (t1.position[1] - t0.position[1]))
    heading = math.degrees(abs(wrap(yaw_of(o1.q) - t1.yaw)))
    print(f"square: turned {turned:.1f} deg, closure {closure:.3f} m, odometry error {error:.4f} m, {heading:.2f} deg")
    if abs(turned - 360.0) > SQUARE_TURN_DEG:
        failures.append(f"square: the sticks turned the vehicle {turned:.0f} deg in all, not a full circle")
    if closure > SQUARE_CLOSURE_M:
        failures.append(f"square: the vehicle ended {closure:.3f} m from where it started")
    if error > ODOMETRY_M or heading > ODOMETRY_DEG:
        failures.append(f"square: the odometry ended {error:.3f} m and {heading:.1f} deg from the truth")


def check_takeover(vehicle: OdometryVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    mode = vehicle.monitor.request_mode(TEST, REPLY_S)
    if mode is None or mode.refused:
        failures.append(f"takeover: TEST was not taken: {mode.reason_text if mode else 'no reply'}")
        return
    if not arm(vehicle, failures, "takeover: the host in TEST"):
        return
    send_for(vehicle, lambda: vehicle.send_body(0.2, 0.0, 0.0), 1.5)
    before = vehicle.statuses[-1].setpoints_rejected if vehicle.statuses else 0
    # A pressed while the host keeps sending.
    taken = time.monotonic()
    end = taken + 0.4
    while time.monotonic() < end:
        vehicle.monitor.sim_gamepad(True, buttons=button("A"))
        vehicle.send_body(0.2, 0.0, 0.0)
        vehicle.monitor.run(GAMEPAD_INTERVAL_S)
    while time.monotonic() < taken + 2.0:
        vehicle.monitor.sim_gamepad(True)
        vehicle.send_body(0.2, 0.0, 0.0)
        vehicle.monitor.run(GAMEPAD_INTERVAL_S)
    status = latest_status(vehicle)
    if status is None or status.mode != Mode.MANUAL or not status.armed:
        failures.append("takeover: A did not switch to MANUAL while staying armed")
        return
    rejected = status.setpoints_rejected - before
    if rejected < 10:
        failures.append(f"takeover: only {rejected} of the host's setpoints were counted dropped")
    stopped = [t for t in vehicle.truth if t.timestamp >= vehicle.mcu_us(taken + STOP_S)]
    if not stopped or any(abs(t.velocity[0]) > STOPPED_MS for t in stopped):
        failures.append("takeover: the vehicle did not stop with the sticks centred")
    start = vehicle.truth[-1]
    gamepad.hold(1.5, sticks=(0.0, STICK, 0.0, 0.0))
    if vehicle.truth[-1].velocity[0] < 0.2:
        failures.append(f"takeover: the left stick did not drive: {vehicle.truth[-1].velocity[0]:.3f} m/s")
    if math.hypot(vehicle.truth[-1].position[0] - start.position[0],
                  vehicle.truth[-1].position[1] - start.position[1]) < 0.15:
        failures.append("takeover: the vehicle did not move under the sticks")
    gamepad.hold(0.5)
    disarm(gamepad)


def disarm_after(vehicle: OdometryVehicle, statuses: int):
    return next((s for s in vehicle.statuses[statuses:] if not s.armed), None)


def check_failsafe(vehicle: OdometryVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    for what, lose in (("unplugged", gamepad.unplug), ("silent", gamepad.silence)):
        gamepad.hold(0.3)
        if not take_over_and_arm(vehicle, gamepad, failures, f"failsafe {what}"):
            return
        gamepad.hold(1.0, sticks=(0.0, STICK, 0.0, 0.0))
        statuses = len(vehicle.statuses)
        lost = time.monotonic()
        lose()
        vehicle.monitor.run(1.5)
        disarm = disarm_after(vehicle, statuses)
        if disarm is None or disarm.disarm_reason != REASON["rc lost"]:
            failures.append(f"the gamepad {what} while driving did not disarm with RC_LOST")
            continue
        late = [o for o in vehicle.outputs if o.timestamp >= disarm.timestamp]
        if not late or any(any(d != 0.0 for d in o.output) for o in late):
            failures.append(f"the gamepad {what}, the outputs did not stop at the disarm")
        after_s = (disarm.timestamp - vehicle.mcu_us(lost)) / 1e6
        print(f"failsafe {what}: disarmed {after_s:.3f} s after the gamepad went")
        if what == "silent" and not RC_TIMEOUT_S <= after_s + 0.05 <= RC_TIMEOUT_S + RC_LOST_MARGIN_S:
            failures.append(f"silent: RC_LOST came {after_s:.3f} s after the last report, not after RC_TIMEOUT")


def check_emergency_stop(vehicle: OdometryVehicle, gamepad: Gamepad, failures: list[str]) -> None:
    gamepad.hold(0.3)
    if not take_over_and_arm(vehicle, gamepad, failures, "emergency stop"):
        return
    gamepad.hold(1.0, sticks=(0.0, STICK, 0.0, 0.0))
    engaged = time.monotonic()
    gamepad.hold(0.2, sticks=(0.0, STICK, 0.0, 0.0), buttons=button("B"))
    gamepad.hold(1.0, sticks=(0.0, STICK, 0.0, 0.0))
    late = [o for o in vehicle.outputs if o.timestamp >= vehicle.mcu_us(engaged + ESTOP_REACT_S)]
    if not late or any(any(d != 0.0 for d in o.output) for o in late):
        failures.append("B did not zero the outputs")
    status = latest_status(vehicle)
    if status is None or not status.emergency_stop or not status.armed:
        failures.append("B: vehicle_status does not show the emergency stop engaged and armed")
    gamepad.hold(HOLD_S, buttons=button("Y"))
    status = latest_status(vehicle)
    if status is None or status.emergency_stop:
        failures.append("Y held did not release the emergency stop")
    released = len(vehicle.outputs)
    gamepad.hold(1.0, sticks=(0.0, STICK, 0.0, 0.0))
    if not any(any(d != 0.0 for d in o.output) for o in vehicle.outputs[released:]):
        failures.append("released, the sticks did not drive again")
    statuses = len(vehicle.statuses)
    gamepad.press("B")
    gamepad.hold(ESTOP_DISARM_S + 0.5)
    disarm = disarm_after(vehicle, statuses)
    if disarm is None or disarm.disarm_reason != REASON["emergency stop"]:
        failures.append("the gamepad's emergency stop held 5 s did not disarm with EMERGENCY_STOP")
    gamepad.hold(HOLD_S, buttons=button("Y"))


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
        if not vehicle.connect():
            failures.append("no handshake with SITL")
        else:
            vehicle.monitor.run(2.0 + SETTLED_S)
            gamepad = Gamepad(vehicle)
            check_arming_refused(vehicle, gamepad, failures)
            check_square(vehicle, gamepad, failures)
            check_takeover(vehicle, gamepad, failures)
            check_failsafe(vehicle, gamepad, failures)
            check_emergency_stop(vehicle, gamepad, failures)
    finally:
        vehicle.close()
        sitl.terminate()
        sitl.communicate(timeout=30)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: the remote control drives, takes over, fails safe and stops the vehicle")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
