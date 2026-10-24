# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests of the uploader.py launcher (#92): every open of the port releases
DTR and RTS in one request, so BOOT0 stays low on a download port, and
uploader.py runs with its own arguments."""

import importlib.util
import struct
import termios
from pathlib import Path

import pytest
import serial

TOOL = Path(__file__).resolve().parent.parent / "run_uploader.py"


@pytest.fixture()
def launcher(monkeypatch):
    spec = importlib.util.spec_from_file_location("run_uploader", TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls = []

    def ioctl(fd, request, arg):
        calls.append((fd, request, arg))

    monkeypatch.setattr(module.fcntl, "ioctl", ioctl)
    module.calls = calls
    return module


def test_every_open_releases_dtr_and_rts_in_one_request(launcher, monkeypatch):
    opened = []

    def base_open(self):
        self.fd = 42
        self.is_open = True
        opened.append(self.port)

    monkeypatch.setattr(serial.Serial, "open", base_open)
    port = launcher.ReleasedLinesSerial("/dev/fake", 115200)
    port.is_open = False
    port.open()  # uploader.py closes and reopens the port when it changes baud rate
    lines = struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS)
    assert opened == ["/dev/fake", "/dev/fake"]
    assert launcher.calls == [(42, termios.TIOCMBIC, lines)] * 2


def test_a_port_without_modem_lines_still_opens(launcher, monkeypatch):
    def ioctl(fd, request, arg):
        raise OSError(25, "Inappropriate ioctl for device")  # a pty

    monkeypatch.setattr(launcher.fcntl, "ioctl", ioctl)
    launcher.release_modem_lines(3)


def test_uploader_runs_with_its_arguments_and_the_released_serial(launcher, tmp_path, monkeypatch):
    log = tmp_path / "uploader.log"
    uploader = tmp_path / "uploader.py"
    uploader.write_text(f"""import sys, serial
open({str(log)!r}, 'w').write(' '.join(sys.argv[1:]) + ' ' + serial.Serial.__name__)
""")
    monkeypatch.setattr(serial, "Serial", serial.Serial)  # undone after the test
    assert launcher.main(["run_uploader.py", str(uploader), "--port", "/dev/fake", "fw.apj"]) == 0
    assert log.read_text() == "--port /dev/fake fw.apj ReleasedLinesSerial"


def test_without_uploader_it_is_a_usage_error(launcher, capsys):
    assert launcher.main(["run_uploader.py"]) == 2
    assert "run_uploader.py <uploader.py>" in capsys.readouterr().err
