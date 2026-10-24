#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL fault test (ADR-0005, ADR-0018): the fault stop and the reset cause.

SITL has no real faults, so it is told to raise one: with --fault assert or
--fault fail, a configASSERT fails or fail() is called once an output has
been driven for 0.5 s. Armed in TEST and driving all four wheels, SITL must
end with the fault exit status, after printing the fault line with every
output's duty zero. Then runs started with --reset-cause, as whoever starts
SITL again after a reset gives it, publish the cause on vehicle_status:
after a watchdog or fault reset arming is refused as an unexpected reset,
after a software reset (a deliberate reboot) it is accepted.

    check_fault.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_actuators import REPLY_S, SETPOINT_INTERVAL_S, SETTLED_S, TARGET, TEST, VERIFIED_S, Vehicle, driving  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402
from vklink.enums import Fault, ResetCause  # noqa: E402

# platform::fault_exit_status.
FAULT_EXIT = 12
# platform::fault_after_driven_us, with the time to arm and reach the wheels and a margin.
FAULT_S = 5.0


def start(args, port: int, *options: str) -> subprocess.Popen:
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}", *options],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    return sitl


def stop(sitl: subprocess.Popen) -> None:
    if sitl.poll() is None:
        sitl.terminate()
    sitl.communicate(timeout=30)


def ready(vehicle: Vehicle, failures: list[str], what: str) -> bool:
    """Connected, the board verified, in TEST and actuator power settled."""
    if not vehicle.connect():
        failures.append(f"{what}: no handshake with SITL")
        return False
    deadline = time.monotonic() + VERIFIED_S
    while time.monotonic() < deadline:
        status = vehicle.status()
        if status is not None and status.board_verified:
            break
    mode = vehicle.monitor.request_mode(TEST, REPLY_S)
    if mode is None or mode.refused:
        failures.append(f"{what}: mode TEST was not taken")
        return False
    vehicle.monitor.run(SETTLED_S)
    return True


def check_fault_stop(args, fault: str, failures: list[str]) -> None:
    what = f"--fault {fault}"
    port = free_udp_port()
    sitl = start(args, port, "--fault", fault)
    vehicle = Vehicle(args, port)
    try:
        if not ready(vehicle, failures, what):
            return
        reply = vehicle.monitor.request_arming(True, REPLY_S)
        if reply is None or reply.refused:
            failures.append(f"{what}: arming was refused: {reply.reason_text if reply else 'no reply'}")
            return
        # Drive until SITL ends; a send to a run that ended may be refused.
        deadline = time.monotonic() + FAULT_S
        while sitl.poll() is None and time.monotonic() < deadline:
            try:
                vehicle.send_setpoint((TARGET,) * 4)
                vehicle.monitor.poll(SETPOINT_INTERVAL_S)
            except OSError:
                break
        try:
            out, _ = sitl.communicate(timeout=FAULT_S)
        except subprocess.TimeoutExpired:
            failures.append(f"{what}: SITL did not end with the fault stop")
            return
    finally:
        vehicle.close()
        stop(sitl)
    if not any(driving(output) for output in vehicle.outputs):
        failures.append(f"{what}: the outputs were never driven before the fault")
    if sitl.returncode != FAULT_EXIT:
        failures.append(f"{what}: SITL ended with status {sitl.returncode}, not {FAULT_EXIT}")
    line = re.search(r"^fault kind=(\S+) address=0x[0-9a-f]{8} duty=(\S+)$", out, re.MULTILINE)
    if line is None:
        failures.append(f"{what}: SITL did not print the fault line")
        return
    if line[1] != fault.upper():
        failures.append(f"{what}: the fault line says {line[1]}")
    duties = [float(duty) for duty in line[2].split(",")]
    if len(duties) != 4 or any(duty != 0.0 for duty in duties):
        failures.append(f"{what}: the fault stop left the duties at {line[2]}")


def check_reset_cause(args, option: str, cause: ResetCause, fault: Fault, refused: bool, failures: list[str]) -> None:
    what = f"--reset-cause {option}"
    port = free_udp_port()
    sitl = start(args, port, "--reset-cause", option)
    vehicle = Vehicle(args, port)
    try:
        if not ready(vehicle, failures, what):
            return
        status = vehicle.status()
        if status is None or status.reset_cause != cause or status.fault != fault:
            failures.append(f"{what}: vehicle_status does not report {cause.name} and {fault.name}")
        reply = vehicle.monitor.request_arming(True, REPLY_S)
        if reply is None:
            failures.append(f"{what}: no reply to the arm request")
        elif refused and (not reply.refused or reply.reason_text != "unexpected reset"):
            failures.append(f"{what}: arming was not refused as an unexpected reset")
        elif not refused and reply.refused:
            failures.append(f"{what}: arming was refused: {reply.reason_text}")
    finally:
        vehicle.close()
        stop(sitl)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("--msg-dir", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    for fault in ("assert", "fail"):
        check_fault_stop(args, fault, failures)
    check_reset_cause(args, "watchdog", ResetCause.WATCHDOG, Fault.NONE, True, failures)
    check_reset_cause(args, "fault:assert", ResetCause.FAULT, Fault.ASSERT, True, failures)
    check_reset_cause(args, "software", ResetCause.SOFTWARE, Fault.NONE, False, failures)

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: the fault stop and the reset cause")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
