#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Runs the pinned uploader.py with the port's DTR and RTS released (#92).

Opening a serial port on Linux asserts DTR and RTS, and pyserial keeps them
asserted. On a port wired for one-key download, such as the C50C's SERIAL2,
RTS asserted holds BOOT0 high, so a reset while uploader.py holds the port,
such as the one that follows reboot --bootloader, starts the STM32 system
bootloader instead of the board's bootloader. This launcher replaces
pyserial's Serial with one that releases both lines in one request each time
the port opens, as the Link monitor does: DTR released first, with RTS still
asserted, would itself reset the board. A pty has no modem lines.

    run_uploader.py <uploader.py> <uploader.py's arguments>...
"""

from __future__ import annotations

import fcntl
import runpy
import struct
import sys
import termios

import serial


def release_modem_lines(fd: int) -> None:
    """Releases DTR and RTS together; a port without modem lines is left as it is."""
    try:
        fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    except OSError:
        pass


class ReleasedLinesSerial(serial.Serial):
    """pyserial's Serial, with DTR and RTS released on every open."""

    def open(self) -> None:
        super().open()
        release_modem_lines(self.fd)


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__.strip().splitlines()[-1].strip(), file=sys.stderr)
        return 2
    serial.Serial = ReleasedLinesSerial
    sys.argv = argv[1:]
    runpy.run_path(argv[1], run_name="__main__")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
