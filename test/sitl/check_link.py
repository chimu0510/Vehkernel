#!/usr/bin/env python3
"""L3 SITL Link test (ADR-0018, spec 0003): SITL's Link checked by the Link monitor.

Starts SITL with its UDP Link on a free port, or with --link pty on a pty,
runs the Link monitor in --check mode against it, then checks SITL's report.
Fails when the monitor's check fails (no handshake, a schema mismatch, lost
frames or a missed rate), when a work item, the Link's included, overran
during the check (by the timing statistics the MCU sent over the Link) or
never ran, or when the pty is left behind once SITL has exited.

Overruns are judged over the check alone: SITL runs on a shared host, and a
stall in the seconds around it says nothing about the Link.

    check_link.py build/test/app/vehkernel-sitl tools/link/link_monitor.py \\
        --msg-dir build/test/generated/schema/python --duration 5 --expect-rate imu_sample:0=200
"""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_report import parse_report  # noqa: E402

# Time SITL runs before and after the monitor's check: start-up and handshake.
MARGIN_S = 4.0


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("monitor", help="tools/link/link_monitor.py")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    parser.add_argument("--link", choices=("udp", "pty"), default="udp", help="SITL's Link transport")
    parser.add_argument("--duration", type=float, default=5.0, help="seconds the monitor checks the Link")
    parser.add_argument("--expect-rate", action="append", default=[], metavar="TOPIC:INSTANCE=HZ",
                        help="required rate of a topic instance, passed to the monitor")
    args = parser.parse_args()

    sitl_args = [args.sitl, "--duration", str(args.duration + 2 * MARGIN_S)]
    if args.link == "udp":
        address = f"127.0.0.1:{free_udp_port()}"
        sitl_args += ["--udp", address]
        target = ["--udp", address]
    else:
        sitl_args += ["--link", "pty"]
    sitl = subprocess.Popen(sitl_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    # SITL prints its Link line first; the pty's path is in it.
    first_line = sitl.stdout.readline()
    print(first_line, end="")
    pty_path = None
    if args.link == "pty":
        fields = dict(pair.split("=", 1) for pair in first_line.split()[1:])
        pty_path = fields.get("path")
        if pty_path is None:
            sitl.kill()
            print(f"FAIL: SITL printed no pty path: {first_line!r}\n{sitl.communicate()[1]}", file=sys.stderr)
            return 1
        target = ["--serial", pty_path]
    monitor_args = [sys.executable, args.monitor, *target, "--msg-dir", args.msg_dir,
                    "--handshake-timeout", str(MARGIN_S), "--check", "--duration", str(args.duration)]
    for expectation in args.expect_rate:
        monitor_args += ["--expect-rate", expectation]
    monitor = subprocess.run(monitor_args, capture_output=True, text=True, timeout=args.duration + 30, check=False)
    report, errors = sitl.communicate(timeout=args.duration + 30)

    print(monitor.stdout, end="")
    print(monitor.stderr, end="", file=sys.stderr)
    print(report, end="")
    failures = []
    if monitor.returncode != 0:
        failures.append(f"the Link monitor's check failed with exit code {monitor.returncode}")
    if sitl.returncode != 0:
        failures.append(f"vehkernel-sitl exited with {sitl.returncode}\n{errors}")
    _, work_items = parse_report(report)
    if not any(item["name"].startswith("link") and int(item["runs"]) > 0 for item in work_items):
        failures.append("no Link work item ran")
    for item in work_items:
        if int(item["runs"]) == 0:
            failures.append(f"work item {item['name']} never ran")
    _, reported = parse_report(monitor.stdout)
    if not reported:
        failures.append("the Link carried no work item timing")
    for item in reported:
        if int(item["overruns"]) != 0:
            failures.append(f"work item {item['name']} overran {item['overruns']} times during the check")
    # A transport without a baud rate, UDP or a pty, is never scaled (ADR-0002).
    scales = [line.split("rate_scale=")[1].split()[0] for line in monitor.stdout.splitlines()
              if line.startswith("mcu_link ") and "rate_scale=" in line]
    if not scales or any(scale != "100" for scale in scales):
        failures.append(f"the Link's rate_scale is {scales or 'missing'}, expected 100 over {args.link}")
    if pty_path is not None and os.path.exists(pty_path):
        failures.append(f"{pty_path} is left behind")

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
