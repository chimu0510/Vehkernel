#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL arming test (ADR-0018, spec 0006): modes, arming checks and the emergency stop.

SITL starts with its simulated actuator power off. Each refusal's reason
shows which check stopped it, in the order the commander runs them.
Through the Link monitor's commands: arming is refused in STANDBY, then in
TEST with actuator power off, then while power settles after it is restored;
with power present it is accepted. Disarmed again, it is refused with the
emergency stop engaged from another connection, which vehicle_status
reports; with the power check skipped by COM_ARM_CHECK and power cut again,
it is accepted, and the reply and vehicle_status report the check skipped.

    check_arming.py build/test/app/vehkernel-sitl tools/link/link_monitor.py \\
        --msg-dir build/test/generated/schema/python --param-meta build/test/generated/params/params.json
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_attitude import free_udp_port  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, load_messages  # noqa: E402

HANDSHAKE_S = 4.0
REPLY_S = 3.0
# Actuator power counts as present after this long (Commander::power_settle_us), with a margin.
SETTLED_S = 0.7
# The IMU answers within this long of boot, verifying the board.
VERIFIED_S = 5.0


def cli(args, address: str, *command: str) -> subprocess.CompletedProcess:
    result = subprocess.run(
        [sys.executable, args.monitor, "--udp", address, "--msg-dir", args.msg_dir, "--param-meta", args.param_meta,
         "--handshake-timeout", str(HANDSHAKE_S), *command],
        capture_output=True, text=True, timeout=30, check=False)
    # Quoted, so the refusals the check expects do not read as its own failures.
    print(f"$ {' '.join(command)}  (exit {result.returncode})")
    for line in (result.stdout + result.stderr).splitlines():
        print(f"| {line}")
    return result


def refused_as(result: subprocess.CompletedProcess, reason: str) -> bool:
    return result.returncode == 1 and f"refused: {reason}" in result.stderr


class Connected:
    """A Link monitor in this process. SITL's UDP Link answers whoever said
    hello last, so one connects again after every command line run."""

    def __init__(self, args, port: int):
        self.connection = UdpConnection("127.0.0.1", port)
        self.monitor = LinkMonitor(self.connection, load_messages(args.msg_dir))

    def __enter__(self) -> LinkMonitor | None:
        return self.monitor if self.monitor.handshake(HANDSHAKE_S) else None

    def __exit__(self, *exc):
        self.connection.close()


def latest_status(monitor: LinkMonitor):
    """The newest vehicle_status, after a moment to let one arrive."""
    monitor.run(0.5)
    statuses = monitor.latest("vehicle_status")
    return statuses[0][1] if statuses else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("monitor")
    parser.add_argument("--msg-dir", required=True)
    parser.add_argument("--param-meta", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    port = free_udp_port()
    address = f"127.0.0.1:{port}"
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", address, "--actuator-power-off"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    try:
        # Until the IMU has answered, every request is refused for the board.
        deadline = time.monotonic() + VERIFIED_S
        while True:
            standby = cli(args, address, "arm")
            if not refused_as(standby, "board not verified") or time.monotonic() > deadline:
                break
        if not refused_as(standby, "wrong mode"):
            failures.append("arming in STANDBY was not refused as the wrong mode")

        if cli(args, address, "mode", "test").stdout != "mode=TEST\n":
            failures.append("mode test was not taken")
        if not refused_as(cli(args, address, "arm"), "actuator power off"):
            failures.append("arming with actuator power off was not refused for it")

        # Timing matters here, so the monitor runs in this process.
        with Connected(args, port) as monitor:
            if monitor is None:
                failures.append("no handshake with SITL")
                return report(failures)
            monitor.sim_actuator_power(True)
            settling = monitor.request_arming(True, REPLY_S)
            if settling is None or settling.reason_text != "actuator power off":
                failures.append("arming right after power was restored was not refused while it settles")
            time.sleep(SETTLED_S)
            powered = monitor.request_arming(True, REPLY_S)
            if powered is None or powered.refused:
                failures.append("with actuator power settled, arming was not accepted")
            disarmed = monitor.request_arming(False, REPLY_S)
            if disarmed is None or disarmed.value != 0:
                failures.append("disarming was not taken")

        # The emergency stop, from another connection than the one in control.
        if cli(args, address, "estop", "on").stdout != "emergency_stop=1\n":
            failures.append("estop on was not taken")
        if not refused_as(cli(args, address, "arm"), "emergency stop engaged"):
            failures.append("arming with the emergency stop engaged was not refused for it")
        with Connected(args, port) as monitor:
            status = latest_status(monitor) if monitor else None
        if status is None or not status.emergency_stop:
            failures.append("vehicle_status does not report the emergency stop engaged")
        if cli(args, address, "estop", "off").stdout != "emergency_stop=0\n":
            failures.append("estop off was not taken")

        # The power check skipped: power cut again no longer refuses arming.
        if cli(args, address, "param", "set", "COM_ARM_CHECK", "0").returncode != 0:
            failures.append("COM_ARM_CHECK could not be set to 0")
        with Connected(args, port) as monitor:
            if monitor is None:
                failures.append("no handshake with SITL")
                return report(failures)
            monitor.sim_actuator_power(False)
            skipped = monitor.request_arming(True, REPLY_S)
            status = latest_status(monitor)
            monitor.request_arming(False, REPLY_S)
        if skipped is None or skipped.refused or skipped.skipped != 1:
            failures.append("with the power check skipped, arming was not accepted with the check reported")
        if status is None or status.skipped_checks != 1 or status.actuator_power != 1:
            failures.append("vehicle_status does not report the power check skipped and the power off")
        if status is not None and not status.board_verified:
            failures.append("vehicle_status does not report the board verified")
    finally:
        sitl.terminate()
        sitl.communicate(timeout=30)
    return report(failures)


def report(failures: list[str]) -> int:
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: arming checks and the emergency stop")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
