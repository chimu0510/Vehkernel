#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Serial flasher: install the Vehkernel bootloader and firmware on a board
over its one-key download port, and update the firmware through that
bootloader (ADR-0020, ADR-0023).

The board definition names the port with a download circuit and its kind.
On the C50C V1.1 that port is SERIAL2, USART1 to its own CH9102F, and the
circuit is DTR_RTS: the adapter's RTS drives BOOT0 (asserted: BOOT0 high) and
its DTR, with RTS, drives NRST (DTR released and RTS asserted: in reset), so
the STM32 system bootloader is entered and left without touching the board.

- new-board backs up the whole chip, erases it and writes
  vehkernel_with_bl.hex, the Vehkernel bootloader and the firmware, through
  the system bootloader with stm32flash.
- update asks the firmware over its Link to reboot into the Vehkernel
  bootloader (link_monitor.py reboot --bootloader). When no Link answers, it
  restarts the board through the download circuit instead, once uploader.py
  is waiting, so the bootloader's boot window catches it. Either way the
  pinned ArduPilot uploader.py then writes vehkernel.apj, which the
  bootloader checks and writes to the FIRMWARE region only: stored
  parameters survive.
- backup reads the whole chip, and reset restarts the board.

    tools/flash/flash_serial.py new-board                # back up, erase the whole chip, write all
    tools/flash/flash_serial.py update                   # firmware only, parameters kept
    tools/flash/flash_serial.py backup vendor.bin        # read the whole chip
    tools/flash/flash_serial.py reset                    # restart the board

Exits 0 on success, 1 when stm32flash or uploader.py failed, 2 on a usage or
tool error.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import queue
import struct
import subprocess
import sys
import termios
import threading
import time
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
link_tools_dir = str(REPO / "tools" / "link")
if link_tools_dir not in sys.path:
    sys.path.insert(0, link_tools_dir)

from vklink.bundle import bundle_root  # noqa: E402

UPLOADER = REPO / "tools" / "bootloader" / "external" / "uploader.py"
LINK_MONITOR = REPO / "tools" / "link" / "link_monitor.py"
# Runs uploader.py with the port's DTR and RTS released, so BOOT0 stays low (#92).
RUN_UPLOADER = Path(__file__).resolve().parent / "run_uploader.py"

# How long the Link monitor may take to ask for the bootloader, and uploader.py to update.
LINK_TIMEOUT_S = 20.0
UPLOAD_TIMEOUT_S = 180.0
# What link_monitor.py prints when the firmware refuses the request.
REBOOT_REFUSED = "reboot refused"
# uploader.py prints this once it has loaded the .apj and starts looking for the bootloader.
UPLOADER_WAITING = "waiting for the bootloader"
# uploader.py does not exit when the bootloader's board ID is not the image's: it waits for another board.
UPLOADER_REFUSED = "Firmware not suitable for this board"

# stm32flash -i sequences by download circuit. DTR_RTS: enter the bootloader
# with NRST held low and BOOT0 high, then release NRST, then BOOT0; leave it
# with NRST held low and BOOT0 high, then release both, BOOT0 falling first
# through NRST's RC. Entering with NRST released while BOOT0 falls, as
# '-dtr,rts,,-rts:dtr' does, restarts the firmware instead.
GPIO_SEQUENCES = {"DTR_RTS": "-dtr,rts,,dtr,,-rts:-dtr,rts,,-rts"}


class FlashError(Exception):
    """A problem with the inputs or the tools, not a failed transfer."""


class TransferError(Exception):
    """stm32flash or uploader.py ran and failed."""


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
    def with_bl_hex(self) -> Path:
        return self.image("vehkernel_with_bl.hex")

    @property
    def apj(self) -> Path:
        return self.image("vehkernel.apj")

    @property
    def msg_dir(self) -> Path:
        return self.directory / "generated" / "schema" / "python"


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
    """Restarts the board: NRST held low with BOOT0 high, then both released."""
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


def ask_link(args: argparse.Namespace, build: Build) -> bool:
    """Asks the firmware over its Link to reboot into the bootloader; whether
    it accepted. A refusal, as while armed, stops the update: the board is
    then never restarted another way."""
    command = [sys.executable, str(args.link_monitor), "--serial", args.port, "--baud", str(args.link_baud),
               "--msg-dir", str(build.msg_dir), "reboot", "--bootloader"]
    print(f"Asking the firmware over the Link on {args.port} to reboot into the bootloader", flush=True)
    try:
        result = subprocess.run(command, check=False, timeout=LINK_TIMEOUT_S, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return False
    print(result.stdout + result.stderr, end="", flush=True)
    if REBOOT_REFUSED in result.stderr:
        raise TransferError("the firmware refused to reboot into the bootloader; disarm it first")
    return result.returncode == 0


def run_uploader(args: argparse.Namespace, build: Build, restart: bool) -> None:
    """Runs uploader.py with the .apj until it exits; with `restart`, restarts
    the board through the download circuit once uploader.py waits for it."""
    command = [sys.executable, "-u", str(RUN_UPLOADER), str(args.uploader), "--port", args.port, "--no-extf",
               str(build.apj)]
    print("$ " + " ".join(command[3:]), flush=True)
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    lines: queue.Queue[str | None] = queue.Queue()

    def read() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=read, daemon=True).start()
    deadline = time.monotonic() + UPLOAD_TIMEOUT_S
    refused = timed_out = False
    try:
        while True:
            try:
                line = lines.get(timeout=max(0.0, deadline - time.monotonic()))
            except queue.Empty:
                timed_out = True
                break
            if line is None:
                break
            print(line, end="", flush=True)
            if restart and UPLOADER_WAITING in line:
                restart = False
                print(f"Restarting the board through the {build.circuit} circuit", flush=True)
                reset(args.port, build.circuit)
            if UPLOADER_REFUSED in line:
                refused = True
                break
    finally:
        if process.poll() is None:
            process.kill()
        returncode = process.wait()
    if refused:
        raise TransferError(f"the bootloader refused {build.apj}: it is for another board")
    if timed_out:
        raise TransferError(f"uploader.py did not finish within {UPLOAD_TIMEOUT_S:.0f} s; if no bootloader "
                            "answered, power-cycle the board while update waits for it")
    if returncode != 0:
        raise TransferError(f"uploader.py failed with exit code {returncode}")


def cmd_update(args: argparse.Namespace, build: Build) -> None:
    apj = build.apj
    if not args.uploader.is_file():
        raise FlashError(f"{args.uploader} is missing")
    asked = ask_link(args, build)
    if not asked:
        print("No Link answered; the board is restarted into its bootloader's boot window instead", flush=True)
    print(f"Writing {apj} through the bootloader; parameters are kept", flush=True)
    run_uploader(args, build, restart=not asked)


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
    print(f"Writing {build.with_bl_hex}: the bootloader and the firmware; parameters start empty")
    flasher.write(build.with_bl_hex)


def cmd_backup(args: argparse.Namespace, build: Build, flasher: Stm32Flash) -> None:
    if args.file.exists():
        raise FlashError(f"{args.file} exists")
    print(f"Backing up {build.flash_size // 1024} KB of Flash to {args.file}")
    flasher.read(args.file, build.flash_start, build.flash_size)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Install the Vehkernel bootloader and firmware over a board's "
                                                 "one-key download port, and update the firmware through it.")
    parser.add_argument("--port", default="/dev/ttyACM0",
                        help="the host device of the board's download port (default /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="the STM32 system bootloader's baud rate (default 115200)")
    parser.add_argument("--link-baud", type=int, default=921600,
                        help="the Link's baud rate on the port, for update (default 921600)")
    bundle = bundle_root(Path(__file__))
    default_build = bundle if bundle is not None else Path("build/c50c-v1_1")
    parser.add_argument("--build", type=Path, default=default_build,
                        help="firmware build directory (default build/c50c-v1_1 or bundle)")
    parser.add_argument("--stm32flash", default="stm32flash", help="stm32flash executable")
    parser.add_argument("--uploader", type=Path, default=UPLOADER, help="uploader.py (default: the pinned copy)")
    parser.add_argument("--link-monitor", type=Path, default=LINK_MONITOR, help="link_monitor.py (default: the repository's)")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("update", help="write vehkernel.apj through the Vehkernel bootloader; parameters are kept")
    new_board = commands.add_parser("new-board", help="back up the whole chip, erase it and write the bootloader "
                                                      "and the firmware; parameters start empty")
    new_board.add_argument("--backup", type=Path, help="backup file (default <board>_flash_<time>.bin)")
    new_board.add_argument("--no-backup", action="store_true", help="erase without a backup")
    backup = commands.add_parser("backup", help="read the whole chip into a file")
    backup.add_argument("file", type=Path)
    commands.add_parser("reset", help="restart the board")
    args = parser.parse_args(argv)

    try:
        build = Build(args.build)
        flasher = Stm32Flash(args.stm32flash, args.port, args.baud, build.circuit)
        print(f"{build.board}: using {args.port}; the board's download port is {build.download_port}, "
              f"with a {build.circuit} circuit")
        if args.command == "update":
            cmd_update(args, build)
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
