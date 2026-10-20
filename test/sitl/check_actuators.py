#!/usr/bin/env python3
"""L3 SITL actuator test (ADR-0018, spec 0006): direct-actuator setpoints turn the simulated wheels.

Starts SITL with its UDP Link on a free port and drives it through the Link
monitor library, judging what wheel_encoders, actuator_outputs and
vehicle_status show. In order: arming in STANDBY is refused; in TEST, armed,
a mode request is refused and COM_ARM_CHECK is locked; 10 rad/s on all four
outputs is reached within 1 s to within 5 %; setpoints stopped disarm after
COM_SP_TIMEOUT with every output zero; with a slow MOT_SLEWRATE the outputs
ramp no faster than it and the disarm still zeroes them; actuator power cut
while turning stops
the wheels and disarms with ACTUATOR_POWER_LOST, restoring it does not re-arm
and arming while it settles is refused; the emergency stop engaged from a
second connection zeroes the outputs, released after 2 s resumes from the next
setpoint and not before, and engaged for 5 s disarms with EMERGENCY_STOP and
refuses arming; with COM_ARM_CHECK=0 and power off, arming is accepted with
the check reported skipped and the wheels do not turn; armed with no setpoint,
the vehicle disarms after COM_DISARM_IDLE.

SITL's clock is the host's monotonic clock since boot, so the MCU times of
the host's requests come from the time sync offset. A reaction "in the next
cycle" is judged on actuator_outputs, which arrives at 50 Hz: every sample
later than the request plus the Link's run, two control cycles and some
scheduling slack must show it. The exact cycle is the unit tests' to check.

    check_actuators.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_attitude import free_udp_port  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, load_messages  # noqa: E402
from vklink.session import DISARM_REASONS, MODES, ParamReject, ParamType, ParamValue  # noqa: E402

HANDSHAKE_S = 4.0
REPLY_S = 3.0
# The IMU answers within this long of boot, verifying the board.
VERIFIED_S = 5.0
# Actuator power counts as present after this long (Commander::power_settle_us), with a margin.
SETTLED_S = 0.7
SETPOINT_INTERVAL_S = 0.05  # 20 Hz, as the Link monitor's drive
CYCLE_US = 5_000            # the control cycle, at the IMU's 200 Hz
# From a request leaving the host to the outputs showing it: the Link's 5 ms
# run, two control cycles (the commander, then actuator control) and slack.
REACT_US = 5_000 + 2 * CYCLE_US + 10_000
# actuator_outputs arrives every 20 ms.
SAMPLE_US = 20_000
EMERGENCY_STOP_DISARM_S = 5.0
SP_TIMEOUT_S = 0.5     # COM_SP_TIMEOUT's default
DISARM_IDLE_S = 1.0    # COM_DISARM_IDLE, set to its minimum
TARGET = 10.0
TOLERANCE = 0.05
STOPPED = 0.5          # rad/s
TEST = next(number for number, name in MODES.items() if name == "TEST")
STANDBY = next(number for number, name in MODES.items() if name == "STANDBY")
REASON = {name: number for number, name in DISARM_REASONS.items()}


class Vehicle:
    """One connection to SITL through the Link monitor library, recording what arrives."""

    def __init__(self, args, port: int):
        self.connection = UdpConnection("127.0.0.1", port)
        self.monitor = LinkMonitor(self.connection, load_messages(args.msg_dir))
        self.outputs: list = []
        self.encoders: list = []
        self.statuses: list = []
        self.monitor.listeners.append(self._record)

    def _record(self, name: str, _instance: int, message) -> None:
        if name == "actuator_outputs":
            self.outputs.append(message)
        elif name == "wheel_encoders":
            self.encoders.append(message)
        elif name == "vehicle_status":
            self.statuses.append(message)

    def connect(self) -> bool:
        """The handshake, then a time sync. SITL's UDP Link answers whoever said hello last."""
        if not self.monitor.handshake(HANDSHAKE_S):
            return False
        deadline = time.monotonic() + REPLY_S
        while self.monitor.time_sync is None and time.monotonic() < deadline:
            self.monitor.run(0.1)
        return self.monitor.time_sync is not None

    def close(self) -> None:
        self.connection.close()

    def mcu_us(self, host_s: float) -> int:
        """The MCU time of a host time."""
        return int(host_s * 1e6) + self.monitor.time_sync.clock_offset_us

    def send_setpoint(self, speeds) -> float:
        """Sends one actuator_setpoint; the host time it left."""
        setpoint = self.monitor.messages.BY_NAME["actuator_setpoint"]
        sent = time.monotonic()
        self.monitor.send_topic("actuator_setpoint", setpoint(wheel_speed=tuple(speeds)))
        return sent

    def status(self):
        """The newest vehicle_status, after a moment to let one arrive."""
        self.monitor.run(0.3)
        return self.statuses[-1] if self.statuses else None


def drive(sender: Vehicle, speeds, duration_s: float, receiver: Vehicle | None = None) -> tuple[float, float]:
    """Sends setpoints from `sender` at 20 Hz while `receiver` (by default the
    sender) receives; the host times of the first and the last setpoint."""
    receiver = receiver or sender
    first = last = sender.send_setpoint(speeds)
    end = first + duration_s
    while (now := time.monotonic()) < end:
        if now - last >= SETPOINT_INTERVAL_S:
            last = sender.send_setpoint(speeds)
        receiver.monitor.poll(min(last + SETPOINT_INTERVAL_S, end) - now)
    return first, last


def driving(output) -> bool:
    return any(duty != 0.0 for duty in output.output)


def holds_target(encoders, from_us: int) -> str | None:
    """Why the wheels did not hold TARGET from `from_us` on, or None."""
    samples = [e for e in encoders if e.timestamp >= from_us]
    if not samples:
        return "no wheel_encoders sample"
    for sample in samples:
        if any(abs(speed - TARGET) > TARGET * TOLERANCE for speed in sample.speed):
            return f"wheel speeds {', '.join(f'{s:.2f}' for s in sample.speed)} rad/s"
    return None


def stopped_after(outputs, deadline_us: int) -> str | None:
    """Why actuator_outputs did not show every output zero after `deadline_us`, or None."""
    late = [o for o in outputs if o.timestamp > deadline_us]
    if not late:
        return "no actuator_outputs sample after it"
    still = [o for o in late if driving(o)]
    if still:
        return f"outputs still driven {(still[-1].timestamp - deadline_us) / 1000:.1f} ms after the deadline"
    return None


def arm(vehicle: Vehicle, failures: list[str], what: str) -> bool:
    reply = vehicle.monitor.request_arming(True, REPLY_S)
    if reply is None or reply.refused:
        failures.append(f"{what}: arming was refused: {reply.reason_text if reply else 'no reply'}")
        return False
    return True


def set_param(vehicle: Vehicle, name: str, kind: ParamType, value) -> ParamValue | ParamReject | None:
    return vehicle.monitor.set_param(name, kind, value, REPLY_S)


def check_arming_and_modes(vehicle: Vehicle, failures: list[str]) -> None:
    # Until the IMU has answered, every request is refused for the board.
    deadline = time.monotonic() + VERIFIED_S
    while True:
        reply = vehicle.monitor.request_arming(True, REPLY_S)
        if reply is None or reply.reason_text != "board not verified" or time.monotonic() > deadline:
            break
        vehicle.monitor.run(0.2)
    if reply is None or reply.reason_text != "wrong mode":
        failures.append("arming in STANDBY was not refused as the wrong mode")
    mode = vehicle.monitor.request_mode(TEST, REPLY_S)
    if mode is None or mode.refused:
        failures.append("mode TEST was not taken")
    vehicle.monitor.run(SETTLED_S)
    if not arm(vehicle, failures, "in TEST with power present"):
        return
    standby = vehicle.monitor.request_mode(STANDBY, REPLY_S)
    if standby is None or not standby.refused or standby.reason_text != "armed":
        failures.append("a mode request while armed was not refused")
    locked = set_param(vehicle, "COM_ARM_CHECK", ParamType.INT32, 0)
    if not isinstance(locked, ParamReject) or locked.reason_text != "locked while armed":
        failures.append("COM_ARM_CHECK could be changed while armed")


def check_speed_and_timeout(vehicle: Vehicle, failures: list[str]) -> None:
    first, last = drive(vehicle, (TARGET,) * 4, 1.5)
    problem = holds_target(vehicle.encoders, vehicle.mcu_us(first) + 1_000_000)
    if problem:
        failures.append(f"10 rad/s was not held within 5 % from 1 s on: {problem}")

    # Setpoints stop: the vehicle disarms after COM_SP_TIMEOUT.
    vehicle.monitor.run(SP_TIMEOUT_S + 0.5)
    timeout_us = vehicle.mcu_us(last) + int(SP_TIMEOUT_S * 1e6)
    problem = stopped_after(vehicle.outputs, timeout_us + REACT_US)
    if problem:
        failures.append(f"setpoints stopped, the outputs were not zero after COM_SP_TIMEOUT: {problem}")
    armed = [o for o in vehicle.outputs if not math.isnan(o.target[0])]
    if not armed or armed[-1].timestamp < timeout_us - CYCLE_US - SAMPLE_US:
        failures.append("setpoints stopped, the vehicle disarmed before COM_SP_TIMEOUT")
    status = vehicle.status()
    if status is None or status.armed or status.disarm_reason != REASON["setpoint timeout"]:
        failures.append("setpoints stopped, vehicle_status does not report a setpoint timeout disarm")


SLOW_SLEW = 0.5        # 1/s: from 0 to the 0.2 duty 10 rad/s needs in 0.4 s


def check_slew(vehicle: Vehicle, failures: list[str]) -> None:
    """A setpoint step ramps the outputs no faster than MOT_SLEWRATE; a stop is not slowed."""
    if not isinstance(set_param(vehicle, "MOT_SLEWRATE", ParamType.FLOAT, SLOW_SLEW), ParamValue):
        failures.append("MOT_SLEWRATE could not be set")
        return
    if not arm(vehicle, failures, "before the slew check"):
        return
    start = len(vehicle.outputs)
    _, last = drive(vehicle, (TARGET,) * 4, 1.0)
    samples = vehicle.outputs[start:]
    steepest = max((abs(b.output[0] - a.output[0]) / ((b.timestamp - a.timestamp) * 1e-6)
                    for a, b in zip(samples, samples[1:]) if b.timestamp > a.timestamp), default=0.0)
    if not samples or steepest > SLOW_SLEW * 1.2:
        failures.append(f"output 1 changed at {steepest:.2f}/s, faster than MOT_SLEWRATE {SLOW_SLEW}")
    # Setpoints stop: the disarm still zeroes every output at once.
    vehicle.monitor.run(SP_TIMEOUT_S + 0.5)
    problem = stopped_after(vehicle.outputs, vehicle.mcu_us(last) + int(SP_TIMEOUT_S * 1e6) + REACT_US)
    if problem:
        failures.append(f"with a slow slew rate, the disarm did not zero the outputs at once: {problem}")
    if not isinstance(set_param(vehicle, "MOT_SLEWRATE", ParamType.FLOAT, 2.0), ParamValue):
        failures.append("MOT_SLEWRATE could not be set back")


def check_power_cut(vehicle: Vehicle, failures: list[str]) -> None:
    if not arm(vehicle, failures, "before the power cut"):
        return
    drive(vehicle, (TARGET,) * 4, 1.2)
    cut = time.monotonic()
    vehicle.monitor.sim_actuator_power(False)
    drive(vehicle, (TARGET,) * 4, 0.8)
    cut_us = vehicle.mcu_us(cut)
    problem = stopped_after(vehicle.outputs, cut_us + REACT_US)
    if problem:
        failures.append(f"actuator power cut, the outputs did not stop in the next cycle: {problem}")
    moving = [e for e in vehicle.encoders if e.timestamp >= cut_us + 600_000 and
              any(abs(speed) > STOPPED for speed in e.speed)]
    if moving or not vehicle.encoders:
        failures.append("actuator power cut, the wheels did not stop")
    status = vehicle.status()
    if status is None or status.armed or status.disarm_reason != REASON["actuator power lost"]:
        failures.append("actuator power cut, vehicle_status does not report an ACTUATOR_POWER_LOST disarm")

    vehicle.monitor.sim_actuator_power(True)
    settling = vehicle.monitor.request_arming(True, REPLY_S)
    if settling is None or settling.reason_text != "actuator power off":
        failures.append("arming within 0.5 s of restoring actuator power was not refused")
    status = vehicle.status()
    if status is None or status.armed:
        failures.append("restoring actuator power re-armed the vehicle")
    vehicle.monitor.run(SETTLED_S)


def check_emergency_stop(vehicle: Vehicle, args, port: int, failures: list[str]) -> Vehicle:
    """Returns the second connection, which SITL now answers."""
    second = Vehicle(args, port)
    if not arm(vehicle, failures, "before the emergency stop"):
        return second
    drive(vehicle, (TARGET,) * 4, 1.2)
    vehicle.send_setpoint((TARGET,) * 4)
    if not second.connect():
        failures.append("no handshake with SITL on a second connection")
        return second

    # Engaged from the second connection, then 2 s with no setpoint at all:
    # the setpoint timeout does not count while it is engaged.
    engage = time.monotonic()
    reply = second.monitor.request_estop(True, REPLY_S)
    if reply is None or reply.value != 1:
        failures.append("estop on from a second connection was not taken")
    second.monitor.run(2.0)
    problem = stopped_after(second.outputs, second.mcu_us(engage) + REACT_US)
    if problem:
        failures.append(f"the emergency stop did not zero the outputs in the next cycle: {problem}")
    status = second.status()
    if status is None or not status.armed or not status.emergency_stop:
        failures.append("2 s into the emergency stop, vehicle_status does not report it engaged and armed")

    # Released: nothing turns until the next setpoint, then the wheels resume.
    release = time.monotonic()
    reply = second.monitor.request_estop(False, REPLY_S)
    if reply is None or reply.value != 0:
        failures.append("estop off was not taken")
    second.monitor.run(0.3)
    if any(driving(o) for o in second.outputs if o.timestamp > second.mcu_us(release)):
        failures.append("released, the outputs resumed before the next setpoint")
    first, _ = drive(vehicle, (TARGET,) * 4, 1.3, receiver=second)
    problem = holds_target(second.encoders, second.mcu_us(first) + 1_000_000)
    if problem:
        failures.append(f"released, the wheels did not resume 10 rad/s from the next setpoint: {problem}")

    # Engaged for 5 s, setpoints still arriving: the vehicle disarms.
    engage = time.monotonic()
    if second.monitor.request_estop(True, REPLY_S) is None:
        failures.append("estop on again was not taken")
    statuses = len(second.statuses)
    end = engage + EMERGENCY_STOP_DISARM_S + 1.0
    while time.monotonic() < end and disarm_after_arming(second.statuses[statuses:]) is None:
        drive(vehicle, (TARGET,) * 4, 0.1, receiver=second)
    disarm = disarm_after_arming(second.statuses[statuses:])
    if disarm is None or disarm.disarm_reason != REASON["emergency stop"]:
        failures.append("5 s into the emergency stop, the vehicle did not disarm with EMERGENCY_STOP")
    else:
        # vehicle_status arrives at 5 Hz, so the disarm may show up to 200 ms late.
        after_s = (disarm.timestamp - second.mcu_us(engage)) / 1e6
        if not EMERGENCY_STOP_DISARM_S - 0.01 <= after_s <= EMERGENCY_STOP_DISARM_S + 0.25:
            failures.append(f"the emergency stop disarmed {after_s:.3f} s after it was engaged, not 5 s")
    refused = second.monitor.request_arming(True, REPLY_S)
    if refused is None or refused.reason_text != "emergency stop engaged":
        failures.append("arming with the emergency stop engaged was not refused for it")
    second.monitor.request_estop(False, REPLY_S)
    # Let the setpoints sent while disarmed go stale.
    second.monitor.run(SP_TIMEOUT_S + 0.1)
    return second


def check_power_check_skipped(vehicle: Vehicle, failures: list[str]) -> None:
    if not isinstance(set_param(vehicle, "COM_ARM_CHECK", ParamType.INT32, 0), ParamValue):
        failures.append("COM_ARM_CHECK could not be set to 0")
        return
    vehicle.monitor.sim_actuator_power(False)
    vehicle.monitor.run(0.1)
    reply = vehicle.monitor.request_arming(True, REPLY_S)
    if reply is None or reply.refused or reply.skipped != 1:
        failures.append("with COM_ARM_CHECK=0 and power off, arming was not accepted with the check skipped")
        return
    status = vehicle.status()
    if status is None or status.skipped_checks != 1 or not status.armed:
        failures.append("vehicle_status does not report the power check skipped")
    first, _ = drive(vehicle, (TARGET,) * 4, 1.0)
    moving = [e for e in vehicle.encoders if e.timestamp >= vehicle.mcu_us(first) and
              any(abs(speed) > STOPPED for speed in e.speed)]
    if moving:
        failures.append("with actuator power off, the wheels turned")
    vehicle.monitor.request_arming(False, REPLY_S)
    vehicle.monitor.sim_actuator_power(True)
    set_param(vehicle, "COM_ARM_CHECK", ParamType.INT32, 1)
    vehicle.monitor.run(max(SETTLED_S, SP_TIMEOUT_S + 0.1))


def disarm_after_arming(statuses):
    """The first vehicle_status reporting a disarm after one reported the vehicle armed, or None."""
    armed = next((i for i, s in enumerate(statuses) if s.armed), None)
    return None if armed is None else next((s for s in statuses[armed:] if not s.armed), None)


def check_idle_disarm(vehicle: Vehicle, failures: list[str]) -> None:
    if not isinstance(set_param(vehicle, "COM_DISARM_IDLE", ParamType.FLOAT, DISARM_IDLE_S), ParamValue):
        failures.append("COM_DISARM_IDLE could not be set")
        return
    statuses = len(vehicle.statuses)
    armed_at = time.monotonic()
    if not arm(vehicle, failures, "before the idle disarm"):
        return
    end = armed_at + DISARM_IDLE_S + 1.0
    while time.monotonic() < end and disarm_after_arming(vehicle.statuses[statuses:]) is None:
        vehicle.monitor.run(0.05)
    disarm = disarm_after_arming(vehicle.statuses[statuses:])
    if disarm is None or disarm.disarm_reason != REASON["idle"]:
        failures.append("armed with no setpoint, the vehicle did not disarm as idle")
        return
    after_s = (disarm.timestamp - vehicle.mcu_us(armed_at)) / 1e6
    if not DISARM_IDLE_S - 0.01 <= after_s <= DISARM_IDLE_S + 0.25:
        failures.append(f"the idle disarm came {after_s:.3f} s after arming, not COM_DISARM_IDLE")


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
    vehicle = Vehicle(args, port)
    second = None
    try:
        if not vehicle.connect():
            failures.append("no handshake with SITL")
            return report(failures)
        check_arming_and_modes(vehicle, failures)
        check_speed_and_timeout(vehicle, failures)
        check_slew(vehicle, failures)
        check_power_cut(vehicle, failures)
        second = check_emergency_stop(vehicle, args, port, failures)
        check_power_check_skipped(second, failures)
        check_idle_disarm(second, failures)
    finally:
        vehicle.close()
        if second is not None:
            second.close()
        sitl.terminate()
        sitl.communicate(timeout=30)
    return report(failures)


def report(failures: list[str]) -> int:
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: actuator control, setpoint arbitration and the safety paths")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
