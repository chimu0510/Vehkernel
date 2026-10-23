#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL reboot test (ADR-0018, ADR-0023, spec 0008): the reboot service over the Link.

Through the Link monitor's commands: while armed in TEST, `reboot
--bootloader` is refused as armed and SITL keeps running. Disarmed, it is
accepted, the reply arrives, and SITL then ends with its bootloader exit
status after printing the reset line. A second SITL run ends with the
firmware exit status after a plain `reboot`.

    check_reboot.py build/test/app/vehkernel-sitl tools/link/link_monitor.py \\
        --msg-dir build/test/generated/schema/python --param-meta build/test/generated/params/params.json
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_arming import SETTLED_S, VERIFIED_S, cli, refused_as  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402

# platform::reset_firmware_exit_status and reset_bootloader_exit_status.
FIRMWARE_EXIT = 10
BOOTLOADER_EXIT = 11
# How long SITL may take to reset after it replied (CommanderTask::reboot_delay_us), with a margin.
RESET_S = 5.0


def start(args, address: str) -> subprocess.Popen:
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", address],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    return sitl


def wait_verified(args, address: str) -> None:
    """Until the IMU has answered, arming is refused for the board; mode changes are not."""
    deadline = time.monotonic() + VERIFIED_S
    while refused_as(cli(args, address, "arm"), "board not verified") and time.monotonic() < deadline:
        pass


def arm(args, address: str) -> bool:
    """Arms in TEST, once actuator power has settled (Commander::power_settle_us)."""
    deadline = time.monotonic() + VERIFIED_S
    while True:
        result = cli(args, address, "arm")
        if not refused_as(result, "actuator power off") or time.monotonic() > deadline:
            return result.returncode == 0
        time.sleep(SETTLED_S)


def ended_with(sitl: subprocess.Popen, status: int, target: str, failures: list[str]) -> None:
    try:
        out, _ = sitl.communicate(timeout=RESET_S)
    except subprocess.TimeoutExpired:
        sitl.kill()
        sitl.communicate()
        failures.append(f"SITL did not reset after reboot to the {target}")
        return
    if sitl.returncode != status:
        failures.append(f"SITL ended with status {sitl.returncode}, not {status} for the {target}")
    if f"reset target={target}" not in out:
        failures.append(f"SITL did not print the reset line for the {target}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("monitor")
    parser.add_argument("--msg-dir", required=True)
    parser.add_argument("--param-meta", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    address = f"127.0.0.1:{free_udp_port()}"
    sitl = start(args, address)
    try:
        wait_verified(args, address)
        if cli(args, address, "mode", "test").stdout != "mode=TEST\n":
            failures.append("mode test was not taken")
        if not arm(args, address):
            failures.append("arming in TEST was not accepted")
        if not refused_as(cli(args, address, "reboot", "--bootloader"), "armed"):
            failures.append("reboot --bootloader while armed was not refused as armed")
        if sitl.poll() is not None:
            failures.append("SITL ended after a refused reboot")
        if cli(args, address, "disarm").returncode != 0:
            failures.append("disarming was not taken")
        if cli(args, address, "reboot", "--bootloader").stdout != "reboot=bootloader\n":
            failures.append("reboot --bootloader while disarmed was not accepted")
        ended_with(sitl, BOOTLOADER_EXIT, "bootloader", failures)
    finally:
        if sitl.poll() is None:
            sitl.terminate()
            sitl.communicate(timeout=30)

    sitl = start(args, address)
    try:
        if cli(args, address, "reboot").stdout != "reboot=firmware\n":
            failures.append("a plain reboot was not accepted")
        ended_with(sitl, FIRMWARE_EXIT, "firmware", failures)
    finally:
        if sitl.poll() is None:
            sitl.terminate()
            sitl.communicate(timeout=30)

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: the reboot service")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
