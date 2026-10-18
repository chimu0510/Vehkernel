#!/usr/bin/env python3
"""L3 SITL smoke test (ADR-0018): run SITL for a fixed time and check its report.

Fails when any work item overran or never ran, or when a topic instance given
with --expect-rate is published at a rate outside the tolerance.

    check_report.py build/sitl/app/vehkernel-sitl --duration 5 \\
        --expect-rate imu_sample:0=200
"""

from __future__ import annotations

import argparse
import subprocess
import sys


def parse_report(text: str) -> tuple[dict[tuple[str, int], dict[str, str]], list[dict[str, str]]]:
    topics: dict[tuple[str, int], dict[str, str]] = {}
    work_items: list[dict[str, str]] = []
    for line in text.splitlines():
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("--duration", type=float, default=3.0, help="run time in seconds")
    parser.add_argument("--expect-rate", type=parse_expectation, action="append", default=[],
                        metavar="TOPIC:INSTANCE=HZ", help="required publish rate of a topic instance")
    parser.add_argument("--tolerance", type=float, default=0.05, help="allowed relative rate error")
    args = parser.parse_args()

    # Any free port for the Link, so the run never collides with another SITL.
    result = subprocess.run([args.sitl, "--duration", str(args.duration), "--udp", "127.0.0.1:0"],
                            capture_output=True, text=True,
                            timeout=args.duration + 30, check=False)
    print(result.stdout, end="")
    if result.returncode != 0:
        print(f"FAIL: vehkernel-sitl exited with {result.returncode}\n{result.stderr}", file=sys.stderr)
        return 1

    topics, work_items = parse_report(result.stdout)
    failures = []
    if not work_items:
        failures.append("the report lists no work items")
    for item in work_items:
        if int(item["overruns"]) != 0:
            failures.append(f"work item {item['name']} overran {item['overruns']} times")
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
