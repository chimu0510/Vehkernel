#!/usr/bin/env python3
"""Link monitor: watch the topics a Vehkernel Link sends (spec 0003).

Connects over UDP (SITL's default) or a serial port (a board's USB serial
port, or SITL's pty), performs the handshake and time synchronization and
reports topics whose schema hash differs from the local build. Without
--check it shows a live view until interrupted: every topic's rate and latest
values, the Link statistics of both ends and every work item's timing (the
summary lines once a second when the output is not a terminal). With
--check it runs for a fixed time, prints a parseable summary and exits
non-zero when the handshake fails, a schema hash differs, frames are lost or
an expected rate is not met.

    link_monitor.py --udp 127.0.0.1:15600
    link_monitor.py --serial /dev/ttyACM0
    link_monitor.py --udp 127.0.0.1:15600 --check --duration 5 --expect-rate imu_sample:0=200
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from vklink.monitor import (  # noqa: E402
    LinkMonitor,
    RateExpectation,
    SerialConnection,
    UdpConnection,
    check,
    load_messages,
)
from vklink.view import render  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
REFRESH_S = 0.5
# ANSI: cursor home, clear the screen.
CLEAR = "\x1b[H\x1b[2J"
DEFAULT_MSG_DIR = REPO_ROOT / "build" / "sitl" / "generated" / "schema" / "python"


def parse_address(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    if not host or not port.isdigit() or not 0 < int(port) < 65536:
        raise argparse.ArgumentTypeError(f"expected <address>:<port>, got '{text}'")
    return host, int(port)


def parse_expectation(text: str) -> RateExpectation:
    try:
        return RateExpectation.parse(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--udp", type=parse_address, default=("127.0.0.1", 15600), metavar="ADDRESS:PORT",
                        help="the Link's UDP address (default 127.0.0.1:15600)")
    target.add_argument("--serial", metavar="PATH", help="the Link's serial port, such as /dev/ttyACM0 or a pty")
    parser.add_argument("--baud", type=int, default=921600, help="serial baud rate (default 921600)")
    parser.add_argument("--msg-dir", default=os.environ.get("VK_MSG_PYTHON_DIR", DEFAULT_MSG_DIR),
                        help="directory of the generated vkmsg.py (default: the sitl build)")
    parser.add_argument("--handshake-timeout", type=float, default=5.0, help="seconds to wait for the MCU")
    parser.add_argument("--check", action="store_true", help="run for --duration, then judge the Link")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds to run with --check")
    parser.add_argument("--expect-rate", type=parse_expectation, action="append", default=[],
                        metavar="TOPIC:INSTANCE=HZ", help="required rate of a topic instance, with --check")
    parser.add_argument("--tolerance", type=float, default=0.05, help="allowed relative rate error")
    args = parser.parse_args()

    try:
        messages = load_messages(args.msg_dir)
    except FileNotFoundError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    try:
        if args.serial:
            connection = SerialConnection(args.serial, args.baud)
            target = f"serial {args.serial} {args.baud} baud"
        else:
            connection = UdpConnection(*args.udp)
            target = "udp {}:{}".format(*args.udp)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    monitor = LinkMonitor(connection, messages)
    try:
        if not monitor.handshake(args.handshake_timeout):
            print("\n".join(monitor.summary()))
            print(f"FAIL: no handshake with the MCU within {args.handshake_timeout:g} s", file=sys.stderr)
            return 1
        if args.check:
            monitor.run(args.duration)
            print("\n".join(monitor.summary()))
            result = check(monitor, args.expect_rate, args.tolerance)
            for failure in result.failures:
                print(f"FAIL: {failure}", file=sys.stderr)
            return 0 if result.passed else 1
        live = sys.stdout.isatty()
        if not live:
            for mismatch in monitor.mismatches:
                print(f"warning: topic {mismatch.name} has a different schema hash on the MCU", file=sys.stderr)
        while True:
            monitor.run(REFRESH_S if live else 1.0)
            if live:
                width = shutil.get_terminal_size().columns
                print(CLEAR + "\n".join(render(monitor, target, width)), flush=True)
            else:
                print("\n".join(monitor.summary()), flush=True)
    except KeyboardInterrupt:
        return 0
    finally:
        connection.close()


if __name__ == "__main__":
    sys.exit(main())
