# SPDX-License-Identifier: GPL-3.0-or-later
"""Direct-actuator setpoints (spec 0006): inbound topic data, the monitor's drive and the drive command."""

import math
import struct
import subprocess
import sys
import types
from pathlib import Path

import pytest

from test_monitor import FakeMcu, Sample
from test_params import MONITOR
from vklink import FrameReader, HostSession, McuControl, McuSession, TopicDescription
from vklink.monitor import LinkMonitor, TopicStats, UdpConnection
from vklink.view import render

SETPOINT_ID = 7
STATUS_ID = 8
SETPOINT = TopicDescription(SETPOINT_ID, 1, 0xB87642E9C9631930, "actuator_setpoint")
STATUS = TopicDescription(STATUS_ID, 1, 0x5555, "vehicle_status")


class ActuatorSetpoint:
    NAME = "actuator_setpoint"
    TOPIC_ID = SETPOINT_ID
    SCHEMA_HASH = SETPOINT.schema_hash
    ENCODED_SIZE = 24
    _STRUCT = struct.Struct("<Q4f")

    def __init__(self, timestamp=0, wheel_speed=(0.0, 0.0, 0.0, 0.0)):
        self.timestamp, self.wheel_speed = timestamp, wheel_speed

    def encode(self) -> bytes:
        return self._STRUCT.pack(self.timestamp, *self.wheel_speed)

    @classmethod
    def decode(cls, data: bytes) -> "ActuatorSetpoint":
        values = cls._STRUCT.unpack(data)
        return cls(values[0], tuple(values[1:]))


class VehicleStatus:
    """Just the fields drive() looks at."""

    NAME = "vehicle_status"
    TOPIC_ID = STATUS_ID
    SCHEMA_HASH = STATUS.schema_hash
    ENCODED_SIZE = 10
    _STRUCT = struct.Struct("<QBB")

    def __init__(self, timestamp=0, armed=False, disarm_reason=0):
        self.timestamp, self.armed, self.disarm_reason = timestamp, armed, disarm_reason

    def encode(self) -> bytes:
        return self._STRUCT.pack(self.timestamp, self.armed, self.disarm_reason)

    @classmethod
    def decode(cls, data: bytes) -> "VehicleStatus":
        return cls(*cls._STRUCT.unpack(data))


def messages():
    classes = (Sample, ActuatorSetpoint, VehicleStatus)
    return types.SimpleNamespace(TOPICS={c.TOPIC_ID: c for c in classes}, BY_NAME={c.NAME: c for c in classes})


def setpoints(mcu: McuSession) -> list[tuple[float, ...]]:
    return [ActuatorSetpoint.decode(message).wheel_speed
            for topic_id, _, message in mcu.topic_data if topic_id == SETPOINT_ID]


def test_topic_data_goes_by_the_id_the_mcu_announced():
    mcu = McuSession([SETPOINT])
    host = HostSession()
    for frame in FrameReader().feed(host.make_hello()):
        for reply in mcu.handle(frame, now_us=1000):
            for answer in FrameReader().feed(reply):
                host.handle_frame(answer)
    assert host.handshake_done
    for frame in FrameReader().feed(host.make_topic_data("actuator_setpoint", b"\x01\x02", instance=0)):
        mcu.handle(frame, now_us=2000)
    assert mcu.topic_data == [(SETPOINT_ID, 0, b"\x01\x02")]
    with pytest.raises(KeyError):
        host.make_topic_data("not_announced", b"")


def test_drive_sends_setpoints_at_20_hz_then_all_nan():
    with FakeMcu(topics=[SETPOINT]) as mcu:
        connection = UdpConnection("127.0.0.1", mcu.port)
        monitor = LinkMonitor(connection, messages())
        try:
            assert monitor.handshake(2.0)
            assert monitor.drive((10.0, -10.0, math.nan, 0.0), 0.5) is None
            monitor.run(0.1)
        finally:
            connection.close()
    sent = setpoints(mcu.session)
    assert 9 <= len(sent) <= 12
    assert sent[0][:2] == (10.0, -10.0) and math.isnan(sent[0][2]) and sent[0][3] == 0.0
    assert all(math.isnan(speed) for speed in sent[-1])


class ScriptedLink:
    """A connection to an McuSession in memory that also sends scripted vehicle_status messages."""

    def __init__(self, statuses: list[VehicleStatus]):
        self.mcu = McuSession([SETPOINT, STATUS])
        self.reader = FrameReader()
        self.pending = b""
        self.statuses = statuses

    def send(self, data: bytes) -> None:
        for frame in self.reader.feed(data):
            self.pending += b"".join(self.mcu.handle(frame, now_us=1000))

    def receive(self, timeout_s: float) -> bytes:
        data, self.pending = self.pending, b""
        if self.mcu.handshake_done and self.statuses and setpoints(self.mcu):
            data += self.mcu.send_topic(STATUS_ID, 0, self.statuses.pop(0).encode(), now_us=1000)
        return data

    def close(self) -> None:
        pass


def test_drive_stops_when_the_vehicle_disarms_after_it_was_armed():
    # A stale disarmed status from before arming does not count.
    link = ScriptedLink([VehicleStatus(1, False, 1), VehicleStatus(2, True), VehicleStatus(3, False, 4)])
    monitor = LinkMonitor(link, messages())
    assert monitor.handshake(1.0)
    disarmed = monitor.drive((5.0, 5.0, 5.0, 5.0), 3.0)
    assert disarmed is not None and disarmed.disarm_reason == 4
    assert all(math.isnan(speed) for speed in setpoints(link.mcu)[-1])


def test_summary_and_live_view_show_wheels_and_outputs():
    connection = types.SimpleNamespace(send=lambda data: None, receive=lambda timeout: b"", close=lambda: None)
    monitor = LinkMonitor(connection, messages())
    encoders = types.SimpleNamespace(timestamp=1, angle=(1.0, 2.0, 3.0, 4.0), speed=(10.0, 9.5, -1.0, 0.0),
                                     valid=0xF)
    outputs = types.SimpleNamespace(timestamp=1, output=(0.2, 0.19, 0.0, 0.0),
                                    target=(10.0, 10.0, math.nan, math.nan))
    for name, message in (("wheel_encoders", encoders), ("actuator_outputs", outputs)):
        monitor.topics[(name, 0)] = TopicStats(name, 0)
        monitor.topics[(name, 0)].add(message)
    summary = monitor.summary()
    assert ("wheel_encoders speed=10.000,9.500,-1.000,0.000 angle=1.000,2.000,3.000,4.000 valid=0xF"
            in summary)
    assert "actuator_outputs output=0.200,0.190,0.000,0.000 target=10.000,10.000,nan,nan" in summary
    view = render(monitor, "udp", width=200)
    assert "wheels         speed=10.000,9.500,-1.000,0.000 angle=1.000,2.000,3.000,4.000 valid=0xF" in view
    assert "outputs        output=0.200,0.190,0.000,0.000 target=10.000,10.000,nan,nan" in view


VKMSG = '''
import struct

class Sample:
    NAME = "sample"
    TOPIC_ID = 0
    SCHEMA_HASH = 0x1122334455667788
    ENCODED_SIZE = 12
    FIELDS = ("timestamp", "value")

    def __init__(self, timestamp=0, value=0):
        self.timestamp, self.value = timestamp, value

    @classmethod
    def decode(cls, data):
        return cls(*struct.unpack("<QI", data))

class ActuatorSetpoint:
    NAME = "actuator_setpoint"
    TOPIC_ID = 7
    SCHEMA_HASH = 0xB87642E9C9631930
    ENCODED_SIZE = 24
    FIELDS = ("timestamp", "wheel_speed")

    def __init__(self, timestamp=0, wheel_speed=(0, 0, 0, 0)):
        self.timestamp, self.wheel_speed = timestamp, wheel_speed

    def encode(self):
        return struct.pack("<Q4f", self.timestamp, *self.wheel_speed)

TOPICS = {0: Sample, 7: ActuatorSetpoint}
BY_NAME = {"sample": Sample, "actuator_setpoint": ActuatorSetpoint}
'''


def run_drive(port: int, tmp_path: Path, *arguments: str) -> subprocess.CompletedProcess:
    (tmp_path / "vkmsg.py").write_text(VKMSG, encoding="utf-8")
    return subprocess.run(
        [sys.executable, str(MONITOR), "--udp", f"127.0.0.1:{port}", "--msg-dir", str(tmp_path),
         "--handshake-timeout", "2", "drive", *arguments],
        capture_output=True, text=True, timeout=30, check=False)


def test_cli_drive_takes_test_arms_drives_and_disarms(tmp_path: Path):
    control = McuControl(skipped=1)
    with FakeMcu(control=control, topics=[SETPOINT]) as mcu:
        result = run_drive(mcu.port, tmp_path, "--duration", "0.3", "--", "-5,nan,2.5,0")
    assert result.returncode == 0, result.stderr
    assert result.stdout == "wheel_speed=none armed=0\n"
    assert "warning: arming checks skipped: ACT_POWER" in result.stderr
    assert control.mode == 1 and not control.armed
    sent = setpoints(mcu.session)
    assert len(sent) >= 5
    assert sent[0][0] == -5.0 and math.isnan(sent[0][1]) and sent[0][2:] == (2.5, 0.0)
    assert all(math.isnan(speed) for speed in sent[-1])


def test_cli_drive_sends_nothing_when_arming_is_refused(tmp_path: Path):
    control = McuControl(arming_result=7)
    with FakeMcu(control=control, topics=[SETPOINT]) as mcu:
        result = run_drive(mcu.port, tmp_path, "1,1,1,1", "--duration", "0.3")
    assert result.returncode == 1
    assert "FAIL: arm refused: actuator power off" in result.stderr
    assert setpoints(mcu.session) == []


def test_cli_drive_needs_four_speeds_and_a_duration(tmp_path: Path):
    with FakeMcu(topics=[SETPOINT]) as mcu:
        three = run_drive(mcu.port, tmp_path, "1,1,1", "--duration", "1")
        no_duration = run_drive(mcu.port, tmp_path, "1,1,1,1")
        zero = run_drive(mcu.port, tmp_path, "1,1,1,1", "--duration", "0")
    assert three.returncode == 2 and "expected four wheel speeds" in three.stderr
    assert no_duration.returncode == 2 and "--duration" in no_duration.stderr
    assert zero.returncode == 2 and "expected a positive number of seconds" in zero.stderr

