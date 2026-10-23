#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Bring-up runner (ADR-0018, L4): flash the bring-up build over SWD, read its
result block back and report.

Runs on the development host, where the ST-Link is. Build the image first in
the devcontainer with the c50c-v1_1-bringup preset, then:

    tools/bringup/run_bringup.py build/c50c-v1_1-bringup/app/vehkernel.elf

Exits 0 when every case passed, 1 when a case failed or the result block is
missing or incomplete, 2 on a usage or tool error.
"""

from __future__ import annotations

import argparse
import re
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

SYMBOL = "vk_bringup_results"
MAGIC = 0x55424B56
VERSION = 1
MAX_CASES = 4
VALUES = 4
HEADER_WORDS = 4
CASE_WORDS = 2 + VALUES
BLOCK_WORDS = HEADER_WORDS + MAX_CASES * CASE_WORDS

STATUS = {0: "NOT RUN", 1: "PASS", 2: "FAIL"}


class RunnerError(Exception):
    """A problem with the tools or inputs, not with the board under test."""


# --- ELF symbol lookup --------------------------------------------------------

def find_symbol(elf: bytes, name: str) -> tuple[int, int]:
    """Address and size of a symbol in a 32-bit little-endian ELF file."""
    if elf[:4] != b"\x7fELF" or elf[4] != 1 or elf[5] != 1:
        raise RunnerError("not a 32-bit little-endian ELF file")
    shoff, = struct.unpack_from("<I", elf, 0x20)
    shentsize, shnum = struct.unpack_from("<HH", elf, 0x2E)
    sections = [struct.unpack_from("<IIIIIIIIII", elf, shoff + i * shentsize) for i in range(shnum)]
    for sh_name, sh_type, _, _, offset, size, link, _, _, entsize in sections:
        if sh_type != 2:  # SHT_SYMTAB
            continue
        strtab = sections[link]
        str_offset = strtab[4]
        for entry in range(size // entsize):
            st_name, st_value, st_size = struct.unpack_from("<III", elf, offset + entry * entsize)
            end = elf.index(b"\0", str_offset + st_name)
            if elf[str_offset + st_name:end].decode() == name:
                return st_value, st_size
    raise RunnerError(f"symbol {name} not found; is this a bring-up build (c50c-v1_1-bringup preset)?")


# --- Result block -------------------------------------------------------------

@dataclass
class Case:
    id: int
    status: int
    values: list[int]


def decode(words: list[int]) -> tuple[bool, list[Case], str]:
    """(complete, cases, problem). `problem` is empty when the block is valid."""
    if len(words) < BLOCK_WORDS:
        return False, [], "result block truncated"
    magic, version, count, complete = words[:HEADER_WORDS]
    if magic != MAGIC:
        return False, [], f"no result block (magic 0x{magic:08X}); did the bring-up firmware start?"
    if version != VERSION:
        return False, [], f"result block version {version}, this runner reads version {VERSION}"
    if count > MAX_CASES:
        return False, [], f"result block claims {count} cases, at most {MAX_CASES} fit"
    cases = []
    for i in range(count):
        base = HEADER_WORDS + i * CASE_WORDS
        values = [v - (1 << 32) if v & 0x80000000 else v for v in words[base + 2:base + CASE_WORDS]]
        cases.append(Case(words[base], words[base + 1], values))
    return complete == 1, cases, ""


def describe(case: Case) -> str:
    v = case.values
    if case.id == 1:
        acked = "acknowledged" if v[1] else "no acknowledge"
        return f"imu_who_am_i     WHO_AM_I = 0x{v[0] & 0xFF:02X} (expected 0xEA), {acked}"
    if case.id == 2:
        return (f"imu_read_rate    mean {v[0] / 1000:.1f} us, longest {v[1] / 1000:.1f} us per read, "
                f"achievable {v[2]} Hz, {v[3]} failed reads")
    return f"case {case.id}         values {v}"


def report(cases: list[Case]) -> bool:
    passed = True
    for case in cases:
        status = STATUS.get(case.status, f"status {case.status}")
        print(f"{status:7}  {describe(case)}")
        passed &= case.status == 1
    return passed


# --- OpenOCD --------------------------------------------------------------------

class OpenOcd:
    def __init__(self, executable: str, configs: list[str]):
        self.base = [executable] + [arg for cfg in configs for arg in ("-f", cfg)]

    def run(self, *commands: str) -> str:
        args = list(self.base)
        for command in commands:
            args += ["-c", command]
        try:
            result = subprocess.run(args, capture_output=True, text=True, timeout=60, check=False)
        except FileNotFoundError:
            raise RunnerError(f"{self.base[0]} not found") from None
        except subprocess.TimeoutExpired:
            raise RunnerError("OpenOCD timed out") from None
        output = result.stdout + result.stderr
        if result.returncode != 0:
            raise RunnerError(f"OpenOCD failed:\n{output}")
        return output

    def flash(self, elf: Path) -> None:
        self.run(f"program {elf} verify reset exit")

    def read_words(self, address: int, count: int) -> list[int]:
        # Halted: with the ST-Link, reads of a running core that sleeps in WFI
        # come back as zeros. The halt lasts only for the read.
        output = self.run("init", "halt", f"echo VKWORDS:[read_memory 0x{address:08X} 32 {count}]", "resume",
                          "exit")
        match = re.search(r"VKWORDS:(.*)", output)
        if match is None:
            raise RunnerError(f"could not read memory:\n{output}")
        return [int(word, 16) for word in match.group(1).split()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Flash the bring-up build and report its results.")
    parser.add_argument("elf", type=Path, help="bring-up image, e.g. build/c50c-v1_1-bringup/app/vehkernel.elf")
    parser.add_argument("--openocd", default="openocd", help="OpenOCD executable")
    parser.add_argument("--config", action="append", default=None,
                        help="OpenOCD configuration file (default: interface/stlink.cfg and target/stm32f4x.cfg)")
    parser.add_argument("--wait", type=float, default=3.0,
                        help="seconds to leave the cases alone before the first read; reading halts the core, "
                             "which would distort timing cases")
    parser.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the cases to finish")
    parser.add_argument("--no-flash", action="store_true", help="read the results of the image already running")
    args = parser.parse_args(argv)

    try:
        address, size = find_symbol(args.elf.read_bytes(), SYMBOL)
        if size < BLOCK_WORDS * 4:
            raise RunnerError(f"{SYMBOL} is {size} bytes, expected {BLOCK_WORDS * 4}")
        openocd = OpenOcd(args.openocd, args.config or ["interface/stlink.cfg", "target/stm32f4x.cfg"])
        if not args.no_flash:
            print(f"Flashing {args.elf} over SWD")
            openocd.flash(args.elf)
        time.sleep(args.wait)
        deadline = time.monotonic() + args.timeout
        while True:
            complete, cases, problem = decode(openocd.read_words(address, BLOCK_WORDS))
            if complete or time.monotonic() >= deadline:
                break
            time.sleep(0.5)
    except (OSError, RunnerError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if problem:
        print(f"FAIL: {problem}")
        return 1
    passed = report(cases)
    if not complete:
        print(f"FAIL: the cases did not finish within {args.timeout:g} s")
        return 1
    print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
