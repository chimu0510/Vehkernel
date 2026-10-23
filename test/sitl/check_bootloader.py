#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL bootloader test (ADR-0018, ADR-0023, spec 0008): ArduPilot's uploader.py against the Vehkernel bootloader.

The SITL bootloader serves the bootloader protocol on a pty, over a file
standing for the FIRMWARE region. The pinned uploader.py
(tools/bootloader/third_party) flashes .apj files built the way the firmware
build builds them (tools/firmware/finalize_image.py):

1. An image for the board flashes, verifies and starts.
2. An image for another board is refused before anything is erased.
3. An upload cut off midway leaves an image that does not start: the next
   boot stays in the bootloader, and a new upload recovers it.
4. RTC_BOOT_HOLD keeps a valid image from starting; RTC_BOOT_FAST starts it
   at once; with no request it starts after the boot window.

    check_bootloader.py build/test/bootloader/vehkernel-sitl-bootloader
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
UPLOADER = REPO / "tools" / "bootloader" / "third_party" / "uploader.py"
sys.path.insert(0, str(REPO / "tools" / "firmware"))

import finalize_image  # noqa: E402

BOARD_ID = 22083
OTHER_BOARD_ID = 30001
REGION_SIZE = 64 * 1024
HOLD = 0xB0070001
FAST = 0xB0070002
UPLOAD_S = 120.0


def image_bytes(size: int = 20000) -> bytearray:
    """Vectors, a descriptor as the compiler emits it, code."""
    image = bytearray(((i * 37) + 11) & 0xFF for i in range(size))
    at = 392
    image[at:at + 36] = finalize_image.SIGNATURE + bytes(16) + bytes([0, 1]) + BOARD_ID.to_bytes(2, "little") + bytes(8)
    return image


def make_apj(path: Path, board_id: int, size: int = 20000) -> bytes:
    image = image_bytes(size)
    image[392 + 26:392 + 28] = board_id.to_bytes(2, "little")
    finalize_image.fill(image, "5212304d")
    board = {"name": "sitl", "board_id": board_id,
             "flash": [{"region": "FIRMWARE", "start": 0x08010000, "size": REGION_SIZE}]}
    path.write_text(finalize_image.apj(bytes(image), board, "0.1", "5212304d"), encoding="utf-8")
    return bytes(image)


class Lines:
    """A process's output lines, collected by a thread so they can be waited for."""

    def __init__(self, process: subprocess.Popen):
        self.process = process
        self.lines: list[str] = []
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self) -> None:
        for line in self.process.stdout:
            self.lines.append(line.rstrip())

    def wait_for(self, text: str, timeout_s: float, start: int = 0) -> bool:
        """Whether a line from `start` on holds `text` before the timeout or the end of the process."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if any(text in line for line in self.lines[start:]):
                return True
            if self.process.poll() is not None and not self._thread.is_alive():
                return any(text in line for line in self.lines[start:])
            time.sleep(0.02)
        return False

    @property
    def text(self) -> str:
        return "\n".join(self.lines)


class Bootloader:
    """A SITL bootloader run."""

    def __init__(self, exe: str, flash: Path, *extra: str):
        self.process = subprocess.Popen(
            [exe, "--flash", str(flash), "--board-id", str(BOARD_ID), "--firmware-size", str(REGION_SIZE), *extra],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        first = self.process.stdout.readline()
        assert first.startswith("bootloader pty="), first
        self.pty = first.strip().split("=", 1)[1]
        self.output = Lines(self.process)

    def read_until(self, text: str, timeout_s: float) -> bool:
        return self.output.wait_for(text, timeout_s)

    @property
    def lines(self) -> list[str]:
        return self.output.lines

    def stop(self) -> None:
        if self.process.poll() is None:
            self.process.kill()
        self.process.wait(timeout=10)


def start_upload(pty: str, apj: Path) -> subprocess.Popen:
    # Unbuffered, so its lines can be watched as they come.
    return subprocess.Popen([sys.executable, "-u", str(UPLOADER), "--port", pty, "--no-extf", str(apj)],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def upload(pty: str, apj: Path, stop_on: str = "") -> tuple[int | None, str]:
    """Runs uploader.py until it exits, or until a line holds `stop_on`; then it is stopped.

    uploader.py does not exit when a board refuses its image: it waits for
    another board, so a refusal is told by its output.
    """
    process = start_upload(pty, apj)
    output = Lines(process)
    if stop_on:
        output.wait_for(stop_on, UPLOAD_S)
    else:
        try:
            process.wait(timeout=UPLOAD_S)
        except subprocess.TimeoutExpired:
            pass
    returncode = process.poll()
    if returncode is None:
        process.kill()
    process.wait(timeout=10)
    output.wait_for("", 0)
    return returncode, output.text


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bootloader")
    args = parser.parse_args()
    failures: list[str] = []
    tmp = Path(tempfile.mkdtemp(prefix="vk-bootloader-"))
    flash = tmp / "firmware.bin"
    good, other = tmp / "good.apj", tmp / "other.apj"
    image = make_apj(good, BOARD_ID)
    make_apj(other, OTHER_BOARD_ID)

    print("1. an image for the board flashes, verifies and starts")
    bl = Bootloader(args.bootloader, str(flash))
    if not bl.read_until("bootloader stay: no_descriptor", 5):
        failures.append("a blank region did not keep the bootloader waiting")
    code, output = upload(bl.pty, good)
    print(output[-300:])
    if code != 0:
        failures.append(f"uploader.py failed: {output[-300:]}")
    if not bl.read_until(f"start size={len(image)}", 5):
        failures.append("the uploaded image did not start")
    bl.stop()
    if flash.read_bytes()[:len(image)] != image:
        failures.append("the region does not hold the uploaded image")

    print("2. an image for another board is refused before anything is erased")
    before = flash.read_bytes()
    bl = Bootloader(args.bootloader, str(flash), "--boot-signal", str(HOLD))
    code, output = upload(bl.pty, other, stop_on="Firmware not suitable for this board")
    if code == 0 or "Firmware not suitable for this board" not in output:
        failures.append(f"uploader.py did not refuse an image for another board: {output[-300:]}")
    bl.stop()
    if flash.read_bytes() != before:
        failures.append("the region changed after a refused upload")

    print("3. an upload cut off midway does not start, and a new upload recovers it")
    bl = Bootloader(args.bootloader, str(flash), "--boot-signal", str(HOLD))
    cut = start_upload(bl.pty, good)
    deadline = time.monotonic() + UPLOAD_S
    # Cut once programming has begun: the region was erased and holds part of the image.
    while time.monotonic() < deadline and cut.poll() is None:
        data = flash.read_bytes()
        if data[:4] == b"\xff" * 4 and data[1000:1004] == image[1000:1004]:
            break
        time.sleep(0.05)
    cut.kill()
    cut.wait(timeout=10)
    bl.stop()  # a power loss
    bl = Bootloader(args.bootloader, str(flash))
    if not bl.read_until("bootloader stay: bad_crc", 5):
        failures.append(f"after a cut-off upload the bootloader did not stay: {bl.lines}")
    code, _ = upload(bl.pty, good)
    if code != 0 or not bl.read_until(f"start size={len(image)}", 5):
        failures.append("a new upload did not recover the board")
    bl.stop()

    print("4. RTC_BOOT_HOLD stays, RTC_BOOT_FAST starts at once, no request starts after the window")
    bl = Bootloader(args.bootloader, str(flash), "--boot-signal", str(HOLD))
    if not bl.read_until("bootloader stay: ok", 5) or bl.read_until("start size=", 2.5):
        failures.append("RTC_BOOT_HOLD did not keep a valid image from starting")
    bl.stop()
    bl = Bootloader(args.bootloader, str(flash), "--boot-signal", str(FAST))
    started = time.monotonic()
    if not bl.read_until("bootloader start at once: ok", 5) or not bl.read_until("start size=", 5) \
            or time.monotonic() - started > 0.8:
        failures.append("RTC_BOOT_FAST did not start the image at once")
    bl.stop()
    bl = Bootloader(args.bootloader, str(flash))
    started = time.monotonic()
    if not bl.read_until("bootloader listen: ok", 5) or not bl.read_until("start size=", 5):
        failures.append("with no request a valid image did not start")
    elif time.monotonic() - started < 0.9:
        failures.append("the image started before the boot window ended")
    bl.stop()

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    if not failures:
        print("PASS: uploader.py flashes the Vehkernel bootloader")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
