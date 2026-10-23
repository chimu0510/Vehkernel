#!/usr/bin/env python3
"""Serial flasher: flash a board through the STM32 system bootloader over
its one-key download port (ADR-0020, ADR-0023).

The board definition names the port with a download circuit and its kind.
On the C50C V1.1 that port is SERIAL2, USART1 to its own CH9102F, and the
circuit is DTR_RTS: the adapter's RTS drives BOOT0 (asserted: BOOT0 high) and
its DTR, with RTS, drives NRST (DTR released and RTS asserted: in reset), so
the bootloader is entered and left without touching the board. stm32flash
does the bootloader protocol; this tool picks what to write, where and with
which DTR/RTS sequence from the build's board definition, so stored
parameters survive an update and a new board loses nothing it had before it
was backed up.

    tools/flash/flash_serial.py update                   # firmware only, parameters kept
    tools/flash/flash_serial.py new-board                # back up, erase the whole chip, write all
    tools/flash/flash_serial.py backup vendor.bin        # read the whole chip
    tools/flash/flash_serial.py reset                    # restart the firmware

Exits 0 on success, 1 when stm32flash failed, 2 on a usage or tool error.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import struct
import subprocess
import sys
import termios
import time
from datetime import datetime
from pathlib import Path

# stm32flash -i sequences by download circuit. DTR_RTS: enter the bootloader
# with NRST held low and BOOT0 high, then release NRST, then BOOT0; leave it
# with NRST held low and BOOT0 high, then release both, BOOT0 falling first
# through NRST's RC. Entering with NRST released while BOOT0 falls, as
# '-dtr,rts,,-rts:dtr' does, restarts the firmware instead.
GPIO_SEQUENCES = {"DTR_RTS": "-dtr,rts,,dtr,,-rts:-dtr,rts,,-rts"}


class FlashError(Exception):
    """A problem with the inputs or the tools, not a failed transfer."""


class TransferError(Exception):
    """stm32flash ran and failed."""


class Build:
    """The images of a firmware build and the Flash layout of its board."""

    def __init__(self, directory: Path):
        self.directory = directory
        board_file = directory / "generated" / "board" / "board.json"
        try:
            board = json.loads(board_file.read_text())
        except OSError as error:
            raise FlashError(f"{board_file}: {error.strerror}; is {directory} a firmware build?") from None
        self.board = board["name"]
        self.regions = {region["region"]: region for region in board["flash"]}
        downloads = [p for p in board.get("serial", []) if p.get("download")]
        if not downloads:
            raise FlashError(f"{board['name']} declares no serial port with a download circuit")
        self.download_port = downloads[0]["port"]
        self.circuit = downloads[0]["download"]
        if self.circuit not in GPIO_SEQUENCES:
            raise FlashError(f"this tool does not know the download circuit {self.circuit}")
        for name in ("BOOT", "FIRMWARE"):
            if name not in self.regions:
                raise FlashError(f"{board['name']} has no {name} Flash region")
        self.flash_start = min(r["start"] for r in self.regions.values())
        self.flash_size = max(r["start"] + r["size"] for r in self.regions.values()) - self.flash_start

    def image(self, name: str) -> Path:
        path = self.directory / "app" / name
        if not path.is_file():
            raise FlashError(f"{path} is missing; build the firmware first")
        return path

    @property
    def firmware_bin(self) -> Path:
        return self.image("vehkernel.bin")

    @property
    def hex(self) -> Path:
        return self.image("vehkernel.hex")

    @property
    def firmware(self) -> dict:
        return self.regions["FIRMWARE"]


class Stm32Flash:
    def __init__(self, executable: str, port: str, baud: int, circuit: str):
        self.base = [executable, "-b", str(baud), "-i", GPIO_SEQUENCES[circuit]]
        self.port = port

    def run(self, *args: str) -> None:
        command = self.base + list(args) + [self.port]
        print("$ " + " ".join(f"'{a}'" if " " in a or "," in a else a for a in command), flush=True)
        try:
            result = subprocess.run(command, check=False)
        except FileNotFoundError:
            raise FlashError(f"{self.base[0]} not found; install it with: sudo apt install stm32flash") from None
        if result.returncode != 0:
            raise TransferError(f"stm32flash failed with exit code {result.returncode}")

    def write(self, image: Path, address: int | None = None) -> None:
        args = ["-w", str(image), "-v"]
        if address is not None:
            args += ["-S", f"0x{address:08X}"]
        self.run(*args)

    def read(self, path: Path, address: int, size: int) -> None:
        self.run("-S", f"0x{address:08X}:{size}", "-r", str(path))

    def erase_all(self) -> None:
        self.run("-o")


def reset(port: str, circuit: str) -> None:
    """Restarts the firmware: NRST held low with BOOT0 high, then both released."""
    assert circuit == "DTR_RTS", circuit
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as error:
        raise FlashError(f"{port}: {error.strerror}") from None
    try:
        def lines(bits: int) -> None:
            fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", bits))

        lines(termios.TIOCM_RTS)  # DTR released, RTS asserted: in reset
        time.sleep(0.1)
        lines(0)
    except OSError as error:
        raise FlashError(f"{port}: cannot drive DTR and RTS ({error.strerror})") from None
    finally:
        os.close(fd)


def default_backup_name(board: str) -> Path:
    return Path(f"{board}_flash_{datetime.now():%Y%m%d_%H%M%S}.bin")


def cmd_update(build: Build, flasher: Stm32Flash) -> None:
    size = build.firmware_bin.stat().st_size
    region = build.firmware
    if size > region["size"]:
        raise FlashError(f"{build.firmware_bin} is {size} bytes, the FIRMWARE region holds {region['size']}")
    print(f"Writing {build.firmware_bin} ({size} bytes) to 0x{region['start']:08X}; parameters are kept")
    flasher.write(build.firmware_bin, region["start"])


def cmd_new_board(args: argparse.Namespace, build: Build, flasher: Stm32Flash) -> None:
    if args.no_backup:
        print("Not backing up: what the board holds now is lost")
    else:
        backup = args.backup or default_backup_name(build.board)
        if backup.exists():
            raise FlashError(f"{backup} exists; pick another name with --backup")
        print(f"Backing up {build.flash_size // 1024} KB of Flash to {backup}")
        flasher.read(backup, build.flash_start, build.flash_size)
        if not backup.is_file() or backup.stat().st_size != build.flash_size:
            raise TransferError(f"the backup {backup} is incomplete; nothing was erased")
    print("Erasing the whole chip")
    flasher.erase_all()
    print(f"Writing {build.hex}: the boot stub and the firmware; parameters start empty")
    flasher.write(build.hex)


def cmd_backup(args: argparse.Namespace, build: Build, flasher: Stm32Flash) -> None:
    if args.file.exists():
        raise FlashError(f"{args.file} exists")
    print(f"Backing up {build.flash_size // 1024} KB of Flash to {args.file}")
    flasher.read(args.file, build.flash_start, build.flash_size)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Flash a board through the STM32 system bootloader over its "
                                                 "one-key download port.")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="the host device of the board's download port (default /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="bootloader baud rate (default 115200)")
    parser.add_argument("--build", type=Path, default=Path("build/c50c-v1_1"),
                        help="firmware build directory (default build/c50c-v1_1)")
    parser.add_argument("--stm32flash", default="stm32flash", help="stm32flash executable")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("update", help="write the firmware from the FIRMWARE region on; parameters are kept")
    new_board = commands.add_parser("new-board", help="back up the whole chip, erase it and write the boot stub "
                                                      "and the firmware; parameters start empty")
    new_board.add_argument("--backup", type=Path, help="backup file (default <board>_flash_<time>.bin)")
    new_board.add_argument("--no-backup", action="store_true", help="erase without a backup")
    backup = commands.add_parser("backup", help="read the whole chip into a file")
    backup.add_argument("file", type=Path)
    commands.add_parser("reset", help="restart the firmware")
    args = parser.parse_args(argv)

    try:
        build = Build(args.build)
        flasher = Stm32Flash(args.stm32flash, args.port, args.baud, build.circuit)
        print(f"{build.board}: download port {build.download_port}, {build.circuit} circuit, on {args.port}")
        if args.command == "update":
            cmd_update(build, flasher)
        elif args.command == "new-board":
            cmd_new_board(args, build, flasher)
        elif args.command == "backup":
            cmd_backup(args, build, flasher)
        else:
            reset(args.port, build.circuit)
    except FlashError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except TransferError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
