#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Finalize a firmware image after linking (ADR-0023).

Fills the app descriptor that the firmware carries in ArduPilot's unsigned
layout, as ArduPilot's set_app_descriptor build step does, in both the
firmware .bin and the .elf, then writes the .apj file ArduPilot's tools read.

    finalize_image.py --elf vehkernel.elf --bin vehkernel.bin \\
        --board-json board.json --apj vehkernel.apj --version 0.1

The .bin is the firmware region only, as the bootloader writes it. The
descriptor's CRCs, image size and git hash are set here; its version and
board ID come from the compiler and are checked against the arguments.
Exits 0 on success and 1 on any problem, leaving the outputs untouched.
"""

from __future__ import annotations

import argparse
import base64
import json
import struct
import subprocess
import sys
import zlib
from pathlib import Path

SIGNATURE = bytes([0x40, 0xA2, 0xE4, 0xF1, 0x64, 0x68, 0x91, 0x06])
DESCRIPTOR_SIZE = 36
# Offsets within the descriptor (core/include/vk/core/app_descriptor.hpp).
CRC1_OFFSET = 8
FILLED_SIZE = 16  # image_crc1, image_crc2, image_size, git_hash
VERSION_OFFSET = 24


class FinalizeError(Exception):
    pass


def crc32(data: bytes, state: int = 0) -> int:
    """ArduPilot's crc32_small: polynomial 0xEDB88320, no final inversion."""
    for byte in data:
        state ^= byte
        for _ in range(8):
            state = (state >> 1) ^ (0xEDB88320 if state & 1 else 0)
    return state


def find_descriptor(image: bytes, what: str) -> int:
    offset = image.find(SIGNATURE)
    if offset < 0:
        raise FinalizeError(f"no app descriptor in the {what}")
    if image.find(SIGNATURE, offset + 1) >= 0:
        raise FinalizeError(f"the app descriptor signature appears twice in the {what}")
    if offset + DESCRIPTOR_SIZE > len(image):
        raise FinalizeError(f"the app descriptor runs past the end of the {what}")
    return offset


def git_identity(source_dir: Path) -> str:
    try:
        result = subprocess.run(["git", "-C", str(source_dir), "rev-parse", "HEAD"],
                                capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return ""
    return result.stdout.strip() if result.returncode == 0 else ""


def fill(image: bytearray, git_hash: str) -> tuple[int, bytes]:
    """Fills the descriptor of a firmware image; returns its offset and the filled bytes."""
    offset = find_descriptor(image, "firmware image")
    start = offset + CRC1_OFFSET
    crc1 = crc32(image[:start])
    crc2 = crc32(image[offset + VERSION_OFFSET:])
    git = int(git_hash[:8], 16) if git_hash else 0
    filled = struct.pack("<IIII", crc1, crc2, len(image), git)
    image[start:start + FILLED_SIZE] = filled
    return offset, filled


def apj(image: bytes, board: dict, version: str, git_hash: str) -> str:
    """The .apj file, with the keys ArduPilot's build writes (Tools/ardupilotwaf/chibios.py)."""
    firmware = next(r for r in board["flash"] if r["region"] == "FIRMWARE")
    data = {
        "board_id": board["board_id"],
        "magic": "APJFWv1",
        "description": f"Vehkernel firmware for a {board['name']} board",
        "image": base64.b64encode(zlib.compress(image, 9)).decode("ascii"),
        "extf_image": base64.b64encode(zlib.compress(b"", 9)).decode("ascii"),
        "summary": board["name"],
        "version": version,
        "image_size": len(image),
        "extf_image_size": 0,
        "flash_total": firmware["size"],
        "image_maxsize": firmware["size"],
        "flash_free": firmware["size"] - len(image),
        "extflash_total": 0,
        "extflash_free": 0,
        "git_identity": git_hash[:8] if git_hash else "unknown",
        "board_revision": 0,
    }
    return json.dumps(data, indent=4) + "\n"


def finalize(elf_path: Path, bin_path: Path, board_json: Path, apj_path: Path, version: str,
             source_dir: Path) -> str:
    board = json.loads(board_json.read_text(encoding="utf-8"))
    if board.get("board_id") is None:
        raise FinalizeError(f"{board_json} gives no board ID")
    image = bytearray(bin_path.read_bytes())
    elf = bytearray(elf_path.read_bytes())

    git_hash = git_identity(source_dir)
    offset, filled = fill(image, git_hash)
    major, minor, board_id = struct.unpack_from("<BBH", image, offset + VERSION_OFFSET)
    if board_id != board["board_id"]:
        raise FinalizeError(f"the image's board ID {board_id} is not the board's {board['board_id']}")
    if f"{major}.{minor}" != version:
        raise FinalizeError(f"the image's version {major}.{minor} is not {version}")
    elf_offset = find_descriptor(elf, "ELF file")
    elf[elf_offset + CRC1_OFFSET:elf_offset + CRC1_OFFSET + FILLED_SIZE] = filled

    apj_text = apj(bytes(image), board, version, git_hash)
    bin_path.write_bytes(image)
    elf_path.write_bytes(elf)
    apj_path.write_text(apj_text, encoding="utf-8")
    crc1, crc2, size, git = struct.unpack("<IIII", filled)
    return (f"app descriptor: board {board_id} version {version} size {size} "
            f"crc1 {crc1:08x} crc2 {crc2:08x} git {git:08x}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Fill the app descriptor and write the .apj file.")
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--bin", type=Path, required=True, help="the firmware region image")
    parser.add_argument("--board-json", type=Path, required=True)
    parser.add_argument("--apj", type=Path, required=True)
    parser.add_argument("--version", required=True, help="major.minor, as the descriptor holds it")
    parser.add_argument("--source-dir", type=Path, default=Path(__file__).resolve().parents[2],
                        help="the git checkout giving the commit")
    args = parser.parse_args(argv)
    try:
        print(finalize(args.elf, args.bin, args.board_json, args.apj, args.version, args.source_dir))
    except (OSError, FinalizeError, KeyError, StopIteration, ValueError) as error:
        print(f"finalize_image.py: error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
