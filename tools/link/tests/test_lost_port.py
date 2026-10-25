# SPDX-License-Identifier: GPL-3.0-or-later
"""The Link monitor reports a port that fails mid-command, as when its USB
cable is pulled while driving, instead of a traceback (found in #50)."""

import importlib.util
import sys
import types
from pathlib import Path

MONITOR = Path(__file__).resolve().parents[1] / "link_monitor.py"


class FakeSerial:
    closed = False

    def __init__(self, path, baud):
        self.path = path

    def close(self):
        FakeSerial.closed = True


def load():
    spec = importlib.util.spec_from_file_location("link_monitor_under_test", MONITOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_a_port_that_fails_while_driving_is_reported(monkeypatch, capsys):
    monitor = load()
    monkeypatch.setattr(monitor, "SerialConnection", FakeSerial)
    monkeypatch.setattr(monitor, "load_messages", lambda directory: types.SimpleNamespace(TOPICS={}))
    monkeypatch.setattr(monitor.LinkMonitor, "handshake", lambda self, timeout: True)

    def pulled_cable(link, args):
        raise OSError(5, "Input/output error")

    monkeypatch.setattr(monitor, "run_drive", pulled_cable)
    monkeypatch.setattr(sys, "argv", ["link_monitor.py", "--serial", "/dev/fake", "drive", "5,5,nan,nan", "--duration", "1"])
    assert monitor.main() == 1
    err = capsys.readouterr().err
    assert "FAIL: the Link's serial /dev/fake 921600 baud failed: Input/output error" in err
    assert "Traceback" not in err
    assert FakeSerial.closed
