#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL test for board examples (spec 0013 Seam C).

Runs a SITL example executable and verifies its report and simulated output.
For 03-oled:
- Verifies the counter rate in the report (10 Hz within tolerance).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last reported counter, every other line blank.
For 04-work-queues:
- Verifies fast (10 Hz) and slow (2 Hz) counter rates over the span between first and last report.
- Verifies no gaps in either counter's sequence.
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last fast counter, line 4 the last slow counter.
For 05-bus:
- Verifies published counter rate (10 Hz) over the span between first and last report.
- Verifies no gaps in published counter sequence.
- Verifies that every received value matches the published ones, each once and in order (allowing 1-2 unreceived at stop).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last received count, line 4 the total received count.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import select
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

from display_text import screen_text  # noqa: E402


def check_03_oled(stdout: str, display_file: Path, duration_s: float, tolerance: float) -> list[str]:
    failures: list[str] = []

    # 1. Check counter rate from stdout report
    counters = [int(m.group(1)) for m in re.finditer(r"^counter=(\d+)", stdout, re.MULTILINE)]
    if not counters:
        failures.append("no counter reported in stdout")
    else:
        for i in range(len(counters) - 1):
            if counters[i + 1] != counters[i] + 1:
                failures.append(f"counter gap: {counters[i]} followed by {counters[i + 1]}")
                break

        measured_rate = len(counters) / duration_s
        expected_rate = 10.0
        if abs(measured_rate - expected_rate) > expected_rate * tolerance:
            failures.append(
                f"counter rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                f"(tolerance {tolerance:.0%})"
            )

    # 2. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 3. Judge whole frames from the captured display image:
    # line 0 the title, line 2 the last reported counter, every other line blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not counters:
                failures.append("cannot verify display content without reported counters")
            else:
                expected_lines = {
                    0: "VEHKERNEL 03-OLED",
                    1: "",
                    2: f"COUNTER: {counters[-1]}",
                    3: "",
                    4: "",
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected_text in expected_lines.items():
                    if lines[line_idx] != expected_text:
                        failures.append(
                            f"display line {line_idx}: expected {expected_text!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_04_work_queues(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Parse counters and arrival timestamps
    fast_reports: list[tuple[int, float]] = []
    slow_reports: list[tuple[int, float]] = []

    for line, t in stdout_lines:
        m_fast = re.match(r"^fast=(\d+)", line)
        if m_fast:
            fast_reports.append((int(m_fast.group(1)), t))
        m_slow = re.match(r"^slow=(\d+)", line)
        if m_slow:
            slow_reports.append((int(m_slow.group(1)), t))

    # 2. Check fast counter: no gaps and rate
    if not fast_reports:
        failures.append("no fast counter reported in stdout")
    else:
        for i in range(len(fast_reports) - 1):
            if fast_reports[i + 1][0] != fast_reports[i][0] + 1:
                failures.append(
                    f"fast counter gap: {fast_reports[i][0]} followed by {fast_reports[i + 1][0]}"
                )
                break

        if len(fast_reports) < 2:
            failures.append("too few fast reports to compute rate")
        else:
            span = fast_reports[-1][1] - fast_reports[0][1]
            if span <= 0:
                failures.append(f"invalid fast time span: {span:.3f} s")
            else:
                measured_rate = (len(fast_reports) - 1) / span
                expected_rate = 10.0
                if abs(measured_rate - expected_rate) > expected_rate * tolerance:
                    failures.append(
                        f"fast rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                        f"(tolerance {tolerance:.0%})"
                    )

    # 3. Check slow counter: no gaps and rate
    if not slow_reports:
        failures.append("no slow counter reported in stdout")
    else:
        for i in range(len(slow_reports) - 1):
            if slow_reports[i + 1][0] != slow_reports[i][0] + 1:
                failures.append(
                    f"slow counter gap: {slow_reports[i][0]} followed by {slow_reports[i + 1][0]}"
                )
                break

        if len(slow_reports) < 2:
            failures.append("too few slow reports to compute rate")
        else:
            span = slow_reports[-1][1] - slow_reports[0][1]
            if span <= 0:
                failures.append(f"invalid slow time span: {span:.3f} s")
            else:
                measured_rate = (len(slow_reports) - 1) / span
                expected_rate = 2.0
                if abs(measured_rate - expected_rate) > expected_rate * tolerance:
                    failures.append(
                        f"slow rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                        f"(tolerance {tolerance:.0%})"
                    )

    # 4. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 5. Judge whole frames from captured display image:
    # line 0 the title, line 2 the last reported fast counter, line 4 the last reported slow counter,
    # every other line blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not fast_reports or not slow_reports:
                failures.append("cannot verify display content without reported counters")
            else:
                expected_lines = {
                    0: "VEHKERNEL 04-WQ",
                    1: "",
                    2: f"FAST: {fast_reports[-1][0]}",
                    3: "",
                    4: f"SLOW: {slow_reports[-1][0]}",
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected_text in expected_lines.items():
                    if lines[line_idx] != expected_text:
                        failures.append(
                            f"display line {line_idx}: expected {expected_text!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def check_05_bus(
    stdout: str,
    stdout_lines: list[tuple[str, float]],
    display_file: Path,
    tolerance: float,
) -> list[str]:
    failures: list[str] = []

    # 1. Parse published and received reports with timestamps
    published_reports: list[tuple[int, float]] = []
    received_reports: list[tuple[int, float]] = []

    for line, t in stdout_lines:
        m_pub = re.match(r"^published=(\d+)", line)
        if m_pub:
            published_reports.append((int(m_pub.group(1)), t))
        m_recv = re.match(r"^received=(\d+)", line)
        if m_recv:
            received_reports.append((int(m_recv.group(1)), t))

    # 2. Check published counter: no gaps and rate
    if not published_reports:
        failures.append("no published counter reported in stdout")
    else:
        for i in range(len(published_reports) - 1):
            if published_reports[i + 1][0] != published_reports[i][0] + 1:
                failures.append(
                    f"published counter gap: {published_reports[i][0]} followed by {published_reports[i + 1][0]}"
                )
                break

        if len(published_reports) < 2:
            failures.append("too few published reports to compute rate")
        else:
            span = published_reports[-1][1] - published_reports[0][1]
            if span <= 0:
                failures.append(f"invalid published time span: {span:.3f} s")
            else:
                measured_rate = (len(published_reports) - 1) / span
                expected_rate = 10.0
                if abs(measured_rate - expected_rate) > expected_rate * tolerance:
                    failures.append(
                        f"publication rate {measured_rate:.2f} Hz outside expected {expected_rate:.2f} Hz "
                        f"(tolerance {tolerance:.0%})"
                    )

    # 3. Check received values: exactly the published ones, each once and in order
    if not received_reports:
        failures.append("no received counter reported in stdout")
    else:
        # Check that the received values are exactly the published ones, each once and in order.
        # Allow the last one or two published just before the stop not to be received yet,
        # because the publisher runs on higher-priority wq:io and may publish just before
        # SITL stops before the subscriber on wq:slow has been scheduled to drain the queue.
        pub_values = [p[0] for p in published_reports]
        recv_values = [r[0] for r in received_reports]

        lag = len(pub_values) - len(recv_values)
        if lag < 0:
            failures.append(
                f"more values received ({len(recv_values)}) than published ({len(pub_values)})"
            )
        elif lag > 2:
            failures.append(
                f"too many published values not received yet: {lag} unreceived (published {len(pub_values)}, "
                f"received {len(recv_values)})"
            )
        else:
            for i, (recv, pub) in enumerate(zip(recv_values, pub_values)):
                if recv != pub:
                    failures.append(
                        f"received value mismatch at index {i}: expected {pub}, got {recv}"
                    )
                    break

    # 4. Check display_on=1 reported from simulated SSD1306's displaying()
    display_on_match = re.search(r"^display_on=(\d+)", stdout, re.MULTILINE)
    if not display_on_match:
        failures.append("display_on not reported in stdout")
    elif int(display_on_match.group(1)) != 1:
        failures.append(f"expected display_on=1, got display_on={display_on_match.group(1)}")

    # 5. Judge whole frames from captured display image:
    # line 0 the title, line 2 the last received count, line 4 the total received count,
    # every other line blank
    if not display_file.exists():
        failures.append("SITL wrote no display image")
    else:
        try:
            lines = screen_text(display_file)
            print("display:")
            for idx, line in enumerate(lines):
                print(f"  [{idx}] |{line}|")

            if not received_reports:
                failures.append("cannot verify display content without reported received counters")
            else:
                expected_lines = {
                    0: "VEHKERNEL 05-BUS",
                    1: "",
                    2: f"COUNT: {received_reports[-1][0]}",
                    3: "",
                    4: f"RECEIVED: {len(received_reports)}",
                    5: "",
                    6: "",
                    7: "",
                }
                for line_idx, expected_text in expected_lines.items():
                    if lines[line_idx] != expected_text:
                        failures.append(
                            f"display line {line_idx}: expected {expected_text!r}, got {lines[line_idx]!r}"
                        )
        except Exception as e:
            failures.append(f"failed to read display image: {e}")

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("executable", help="the example executable to run (e.g. example-03-oled)")
    parser.add_argument("--duration", type=float, default=3.0, help="duration to run SITL in seconds")
    parser.add_argument("--tolerance", type=float, default=0.20, help="allowed relative rate error")
    args = parser.parse_args()

    target = Path(args.executable)
    name = target.name
    if name.startswith("example-"):
        ex_name = name.removeprefix("example-")
    else:
        ex_name = name

    with tempfile.TemporaryDirectory() as tmp:
        display_file = Path(tmp) / "display.pbm"
        cmd = [str(target), "--duration", str(args.duration), "--display", str(display_file)]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        stdout_lines: list[tuple[str, float]] = []
        deadline = time.monotonic() + args.duration + 30
        assert proc.stdout is not None
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                proc.kill()
                raise subprocess.TimeoutExpired(cmd, args.duration + 30)
            rlist, _, _ = select.select([proc.stdout], [], [], remaining)
            if not rlist:
                proc.kill()
                raise subprocess.TimeoutExpired(cmd, args.duration + 30)
            line = proc.stdout.readline()
            if not line:
                break
            t = time.monotonic()
            stdout_lines.append((line, t))
            print(line, end="")

        _, stderr = proc.communicate(timeout=30)
        if proc.returncode != 0:
            print(stderr, file=sys.stderr)
            print(f"FAIL: {name} exited with {proc.returncode}", file=sys.stderr)
            return 1

        full_stdout = "".join(line for line, _ in stdout_lines)

        if ex_name == "03-oled":
            failures = check_03_oled(full_stdout, display_file, args.duration, args.tolerance)
        elif ex_name == "04-work-queues":
            failures = check_04_work_queues(full_stdout, stdout_lines, display_file, args.tolerance)
        elif ex_name == "05-bus":
            failures = check_05_bus(full_stdout, stdout_lines, display_file, args.tolerance)
        else:
            failures = [f"unknown SITL example {ex_name}"]

    if failures:
        for f in failures:
            print(f"FAIL: {f}", file=sys.stderr)
        return 1

    print(f"PASS: {name} completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
