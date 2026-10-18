"""The Link monitor against a fake MCU over UDP on the loopback (spec 0003)."""

import socket
import struct
import threading
import types

import pytest

from vklink import FrameReader, McuSession, TopicDescription
from vklink.monitor import LinkMonitor, RateExpectation, UdpConnection, check

SAMPLE_HASH = 0x1122334455667788


class Sample:
    NAME = "sample"
    TOPIC_ID = 0
    SCHEMA_HASH = SAMPLE_HASH
    ENCODED_SIZE = 12
    _STRUCT = struct.Struct("<QI")

    def __init__(self, timestamp=0, value=0):
        self.timestamp = timestamp
        self.value = value

    def encode(self) -> bytes:
        return self._STRUCT.pack(self.timestamp, self.value)

    @classmethod
    def decode(cls, data: bytes) -> "Sample":
        return cls(*cls._STRUCT.unpack(data))


def messages(schema_hash=SAMPLE_HASH):
    """A stand-in for the generated vkmsg module."""
    cls = type("Sample", (Sample,), {"SCHEMA_HASH": schema_hash})
    return types.SimpleNamespace(TOPICS={0: cls}, BY_NAME={"sample": cls})


class FakeMcu:
    """Answers the handshake and time sync, and sends `sample` every 10 ms of MCU time."""

    def __init__(self, skip_sequence: bool = False):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.002)
        self.port = self.socket.getsockname()[1]
        self.session = McuSession([TopicDescription(0, 1, SAMPLE_HASH, "sample")])
        self.reader = FrameReader()
        self.skip_sequence = skip_sequence
        self.peer = None
        self.now_us = 1_000_000
        self.sent = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join()
        self.socket.close()

    def _loop(self):
        while not self._stop.is_set():
            try:
                data, sender = self.socket.recvfrom(65536)
                for frame in self.reader.feed(data):
                    self.peer = sender
                    for reply in self.session.handle(frame, self.now_us):
                        self.socket.sendto(reply, sender)
            except TimeoutError:
                pass
            self.now_us += 10_000
            wire = self.session.send_topic(0, 0, Sample(self.now_us, self.sent).encode(), self.now_us)
            if wire is None:
                continue
            self.sent += 1
            if self.skip_sequence and self.sent == 5:
                continue  # Lost on the way.
            self.socket.sendto(wire, self.peer)


def run_monitor(mcu: FakeMcu, schema=None, duration_s: float = 0.5) -> LinkMonitor:
    connection = UdpConnection("127.0.0.1", mcu.port)
    monitor = LinkMonitor(connection, schema or messages())
    try:
        assert monitor.handshake(2.0)
        monitor.run(duration_s)
    finally:
        connection.close()
    return monitor


def test_handshake_and_rate_check_pass():
    with FakeMcu() as mcu:
        monitor = run_monitor(mcu)
    result = check(monitor, [RateExpectation("sample", 0, 100.0)], tolerance=0.05)
    assert result.passed, result.failures
    assert monitor.topics[("sample", 0)].messages > 10
    summary = monitor.summary()
    assert summary[0].startswith("link handshake=ok version=1 mismatches=0")
    assert any(line.startswith("topic name=sample instance=0") and "rate_hz=100.00" in line for line in summary)
    assert monitor.time_sync is not None


def test_schema_mismatch_fails_the_check_and_names_the_topic():
    with FakeMcu() as mcu:
        monitor = run_monitor(mcu, messages(schema_hash=0xDEAD))
    result = check(monitor, [], tolerance=0.05)
    assert not result.passed
    assert "topic sample has a different schema hash on the MCU" in result.failures
    assert any(line.startswith("mismatch topic=sample") for line in monitor.summary())


def test_lost_frames_fail_the_check():
    with FakeMcu(skip_sequence=True) as mcu:
        monitor = run_monitor(mcu)
    result = check(monitor, [], tolerance=0.05)
    assert "1 frames lost" in result.failures


def test_a_missed_rate_fails_the_check():
    with FakeMcu() as mcu:
        monitor = run_monitor(mcu)
    result = check(monitor, [RateExpectation("sample", 0, 200.0), RateExpectation("other", 0, 1.0)], 0.05)
    assert any(failure.startswith("topic sample:0 arrived at 100.00 Hz") for failure in result.failures)
    assert "topic other:0 never arrived" in result.failures


def test_no_mcu_means_no_handshake():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as silent:
        silent.bind(("127.0.0.1", 0))
        connection = UdpConnection("127.0.0.1", silent.getsockname()[1])
        monitor = LinkMonitor(connection, messages())
        assert not monitor.handshake(0.3)
        connection.close()
    assert check(monitor, [], 0.05).failures == ["no handshake with the MCU"]


def test_rate_expectation_syntax():
    assert RateExpectation.parse("imu_sample:0=200") == RateExpectation("imu_sample", 0, 200.0)
    with pytest.raises(ValueError):
        RateExpectation.parse("imu_sample=200")
