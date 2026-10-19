#!/usr/bin/env python3
"""L3 SITL smoke test (ADR-0018): run SITL for a fixed time and check its report.

Fails when any work item overran or never ran, or when a topic instance given
with --expect-rate is published at a rate outside the tolerance.

With --msg-dir, overruns are judged over a check window alone, by the timing
statistics the MCU sends over its UDP Link: the overruns at the window's end
less those at its start. SITL runs on a shared host, and a stall around
start-up says nothing about the work items. Without it, over SITL's whole run.

    check_report.py build/sitl/app/vehkernel-sitl --duration 5 \\
        --msg-dir build/sitl/generated/schema/python --expect-rate imu_sample:0=200
"""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
from pathlib import Path

# Time SITL runs before and after the check window: start-up and handshake.
MARGIN_S = 2.0


def parse_report(text: str) -> tuple[dict[tuple[str, int], dict[str, str]], list[dict[str, str]]]:
    topics: dict[tuple[str, int], dict[str, str]] = {}
    work_items: list[dict[str, str]] = []
    for line in text.splitlines():
        if not line.strip():
            continue
        kind, *pairs = line.split()
        fields = dict(pair.split("=", 1) for pair in pairs)
        if kind == "topic":
            topics[(fields["name"], int(fields["instance"]))] = fields
        elif kind == "work_item":
            work_items.append(fields)
    return topics, work_items


def parse_expectation(text: str) -> tuple[str, int, float]:
    try:
        target, rate = text.split("=")
        name, instance = target.split(":")
        return name, int(instance), float(rate)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected <topic>:<instance>=<rate_hz>, got '{text}'") from None


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def window_overruns(port: int, msg_dir: str, duration_s: float) -> tuple[dict[str, int], list[str]]:
    """Overruns of every work item during a window of the Link, by name."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools" / "link"))
    from vklink.monitor import LinkMonitor, UdpConnection, load_messages, work_item_name

    connection = UdpConnection("127.0.0.1", port)
    monitor = LinkMonitor(connection, load_messages(msg_dir))
    try:
        if not monitor.handshake(MARGIN_S):
            return {}, ["no handshake with SITL"]
        # Let every work item send its status once before the window opens.
        monitor.run(MARGIN_S / 2)
        start = {work_item_name(status): status.overruns for _, status in monitor.latest("work_item_status")}
        monitor.run(duration_s)
        end = {work_item_name(status): status.overruns for _, status in monitor.latest("work_item_status")}
    finally:
        connection.close()
    if not end:
        return {}, ["the Link carried no work item timing"]
    return {name: overruns - start.get(name, 0) for name, overruns in end.items()}, []


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="run time in seconds; with --msg-dir, the check window's")
    parser.add_argument("--msg-dir", help="directory of the generated vkmsg.py; judges overruns over the Link")
    parser.add_argument("--expect-rate", type=parse_expectation, action="append", default=[],
                        metavar="TOPIC:INSTANCE=HZ", help="required publish rate of a topic instance")
    parser.add_argument("--tolerance", type=float, default=0.05, help="allowed relative rate error")
    args = parser.parse_args()

    failures = []
    if args.msg_dir is None:
        # Any free port for the Link, so the run never collides with another SITL.
        result = subprocess.run([args.sitl, "--duration", str(args.duration), "--udp", "127.0.0.1:0"],
                                capture_output=True, text=True,
                                timeout=args.duration + 30, check=False)
        report, errors, returncode = result.stdout, result.stderr, result.returncode
        overruns = None
    else:
        duration = args.duration + 2 * MARGIN_S
        port = free_udp_port()
        sitl = subprocess.Popen([args.sitl, "--duration", str(duration), "--udp", f"127.0.0.1:{port}"],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        link_line = sitl.stdout.readline()  # printed once the Link is open
        overruns, link_failures = window_overruns(port, args.msg_dir, args.duration)
        failures += link_failures
        rest, errors = sitl.communicate(timeout=duration + 30)
        report, returncode = link_line + rest, sitl.returncode
    print(report, end="")
    if returncode != 0:
        print(f"FAIL: vehkernel-sitl exited with {returncode}\n{errors}", file=sys.stderr)
        return 1

    topics, work_items = parse_report(report)
    if not work_items:
        failures.append("the report lists no work items")
    if overruns is None:
        overruns = {item["name"]: int(item["overruns"]) for item in work_items}
    for name, count in sorted(overruns.items()):
        if count != 0:
            failures.append(f"work item {name} overran {count} times"
                            + (" during the check" if args.msg_dir else ""))
    for item in work_items:
        if int(item["runs"]) == 0:
            failures.append(f"work item {item['name']} never ran")
    for name, instance, rate in args.expect_rate:
        topic = topics.get((name, instance))
        if topic is None:
            failures.append(f"topic {name}:{instance} is not in the report")
            continue
        measured = float(topic["rate_hz"])
        if abs(measured - rate) > rate * args.tolerance:
            failures.append(f"topic {name}:{instance} published at {measured:.2f} Hz, expected {rate:g} Hz "
                            f"within {args.tolerance:.0%}")

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
