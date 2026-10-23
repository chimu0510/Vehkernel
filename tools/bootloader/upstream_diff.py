#!/usr/bin/env python3
"""Lists the upstream ArduPilot changes the Vehkernel bootloader may need to follow (ADR-0023).

The bootloader ports the structure of ArduPilot's Tools/AP_Bootloader and
libraries/AP_CheckFirmware at the commit in bootloader/UPSTREAM, and
tools/bootloader/third_party/uploader.py is ArduPilot's Tools/scripts/uploader.py
at the same commit. Given an ArduPilot checkout, this lists every commit since
then that touches those paths, with the files each changed, so the port can
adopt those that change the protocol, the boot decision or the descriptor.

    tools/bootloader/upstream_diff.py ~/ardupilot              # commits since UPSTREAM up to HEAD
    tools/bootloader/upstream_diff.py ~/ardupilot --to master --diff

Exits 0 when nothing changed, 1 when there are changes to review, and 2 when
the checkout or the commits cannot be read.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
UPSTREAM_FILE = REPO / "bootloader" / "UPSTREAM"
PATHS = ("Tools/AP_Bootloader", "libraries/AP_CheckFirmware", "Tools/scripts/uploader.py")


def git(checkout: Path, *args: str) -> str:
    result = subprocess.run(["git", "-C", str(checkout), *args], capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or f"git {' '.join(args)} failed")
    return result.stdout


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("ardupilot", type=Path, help="an ArduPilot git checkout")
    parser.add_argument("--from", dest="base", help="the revision the port follows (default bootloader/UPSTREAM)")
    parser.add_argument("--to", default="HEAD", help="the upstream revision to compare with (default HEAD)")
    parser.add_argument("--diff", action="store_true", help="also print the changes themselves")
    args = parser.parse_args(argv)

    base = args.base or UPSTREAM_FILE.read_text(encoding="utf-8").strip()
    try:
        commits = git(args.ardupilot, "log", "--format=%h %cs %s", f"{base}..{args.to}", "--", *PATHS).splitlines()
        stat = git(args.ardupilot, "diff", "--stat", f"{base}..{args.to}", "--", *PATHS)
        diff = git(args.ardupilot, "diff", f"{base}..{args.to}", "--", *PATHS) if args.diff else ""
    except (OSError, RuntimeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    print(f"Vehkernel bootloader follows ArduPilot {base[:12]}; comparing with {args.to}")
    if not commits:
        print("No upstream changes to the bootloader, the descriptor check or uploader.py.")
        return 0
    print(f"{len(commits)} upstream commits to review:")
    for line in commits:
        print(f"  {line}")
    print(stat, end="")
    print(diff, end="")
    print("After adopting them, set bootloader/UPSTREAM to the new commit, and copy uploader.py and\n"
          "Tools/AP_Bootloader/board_types.txt (tools/generators/vkboard/board_types.txt) again.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
