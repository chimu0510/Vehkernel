#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL display test (ADR-0018, spec 0007): the local display's status page.

Runs SITL with --display for a few seconds and reads the text on the image the
simulated SSD1306 last showed: the status page, in STANDBY and disarmed, with
the board verified and actuator power on.

    check_display.py build/test/app/vehkernel-sitl
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from display_text import screen_text  # noqa: E402

DURATION_S = 3.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        image = Path(tmp) / "display.pbm"
        result = subprocess.run([args.sitl, "--duration", str(DURATION_S), "--udp", "127.0.0.1:0",
                                 "--display", str(image)],
                                capture_output=True, text=True, timeout=30, check=False)
        if result.returncode != 0:
            print(result.stdout, result.stderr, sep="\n")
            print(f"FAIL: SITL exited with {result.returncode}")
            return 1
        if not image.exists():
            print("FAIL: SITL wrote no display image")
            return 1
        lines = screen_text(image)

    print("display:")
    for line in lines:
        print(f"  |{line}|")
    expected = {0: "STATUS            1/3", 1: "STANDBY DISARMED", 2: "ESTOP OFF BOARD OK", 3: "POWER ON"}
    failures = [f"line {n}: expected {text!r}, got {lines[n]!r}" for n, text in expected.items() if lines[n] != text]
    for failure in failures:
        print(f"FAIL: {failure}")
    if not failures:
        print("PASS: the status page shows STANDBY, disarmed, the board verified and actuator power on")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
