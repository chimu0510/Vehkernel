#!/usr/bin/env python3
"""L3 SITL bring-up test (ADR-0018, spec 0006): the motor-to-encoder case.

SITL starts with output 2's motor wired the wrong way round. In TEST and
armed, the Link monitor's bringup motors finds output 2 reversed and the
others passing; after MOT2_REV is set, while disarmed since it is locked
while armed, every output passes. A case is refused while disarmed.

    check_bringup.py build/test/app/vehkernel-sitl tools/link/link_monitor.py \\
        --msg-dir build/test/generated/schema/python --param-meta build/test/generated/params/params.json
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_arming import cli, refused_as  # noqa: E402
from check_attitude import free_udp_port  # noqa: E402

# The board verifies and actuator power settles within this long of boot.
READY_S = 5.0


def arm_when_ready(args, address: str) -> bool:
    deadline = time.monotonic() + READY_S
    while time.monotonic() < deadline:
        armed = cli(args, address, "arm")
        if armed.returncode == 0:
            return True
        time.sleep(0.2)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl")
    parser.add_argument("monitor")
    parser.add_argument("--msg-dir", required=True)
    parser.add_argument("--param-meta", required=True)
    args = parser.parse_args()

    failures: list[str] = []
    address = f"127.0.0.1:{free_udp_port()}"
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", address, "--reversed-motor", "2"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    try:
        if cli(args, address, "mode", "test").returncode != 0:
            failures.append("mode test was not taken")
        if not refused_as(cli(args, address, "bringup", "motors"), "not armed"):
            failures.append("a bring-up case while disarmed was not refused")
        if not arm_when_ready(args, address):
            failures.append("SITL could not be armed")
        found = cli(args, address, "bringup", "motors")
        lines = found.stdout.splitlines()
        if found.returncode != 1 or len(lines) != 4 \
                or not lines[1].startswith("bringup output=2 result=reversed encoder=2 sign=-1") \
                or not all(f"output={n} result=pass encoder={n} sign=+1" in lines[n - 1] for n in (1, 3, 4)):
            failures.append("bringup motors did not find output 2 reversed and the others passing")

        cli(args, address, "disarm")
        if cli(args, address, "param", "set", "MOT2_REV", "REVERSED").returncode != 0:
            failures.append("MOT2_REV could not be set while disarmed")
        if not arm_when_ready(args, address):
            failures.append("SITL could not be armed again")
        fixed = cli(args, address, "bringup", "motors")
        if fixed.returncode != 0 or len(fixed.stdout.splitlines()) != 4:
            failures.append("with MOT2_REV set, not every output passed")
        cli(args, address, "disarm")
    finally:
        sitl.terminate()
        sitl.communicate(timeout=30)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: the bring-up case finds a reversed motor and passes once it is fixed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
