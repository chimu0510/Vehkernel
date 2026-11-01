#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL test for board examples (spec 0013 Seam C).

Runs a SITL example executable and verifies its report and simulated output.
For 03-oled:
- Verifies the counter rate in the report (10 Hz within tolerance).
- Verifies display_on=1 reported from the simulated SSD1306.
- Verifies whole frames: line 0 the title, line 2 the last reported counter, every other line blank.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import sys
import tempfile

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
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=args.duration + 30, check=False)

        print(result.stdout, end="")
        if result.returncode != 0:
            print(result.stderr, file=sys.stderr)
            print(f"FAIL: {name} exited with {result.returncode}", file=sys.stderr)
            return 1

        if ex_name == "03-oled":
            failures = check_03_oled(result.stdout, display_file, args.duration, args.tolerance)
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
