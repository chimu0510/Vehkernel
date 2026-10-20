"""The Link monitor against a fake MCU over UDP on the loopback (spec 0003)."""

import math
import socket
import struct
import threading
import types

import pytest

from vklink import FrameReader, McuSession, TopicDescription
from vklink.monitor import LinkMonitor, RateExpectation, TopicStats, UdpConnection, check, euler_degrees
from vklink.view import render

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

    def __init__(self, skip_sequence: bool = False, params=None, param_set_hash: int = 0, control=None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.002)
        self.port = self.socket.getsockname()[1]
        self.session = McuSession([TopicDescription(0, 1, SAMPLE_HASH, "sample")], params=params,
                                  param_set_hash=param_set_hash, control=control)
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
    assert summary[0].startswith("link handshake=ok version=3 mismatches=0")
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


def test_summary_and_live_view_show_link_statistics_and_work_item_timing():
    connection = types.SimpleNamespace(send=lambda data: None, receive=lambda timeout: b"", close=lambda: None)
    monitor = LinkMonitor(connection, messages())
    monitor.session.handshake_done = True
    monitor.session.version = 1
    link = types.SimpleNamespace(timestamp=1, frames_sent=100, bytes_sent=4000, frames_received=5,
                                 bytes_received=80, drops=2, crc_errors=1, malformed=0, lost=3, handshake_state=1)
    worker = types.SimpleNamespace(timestamp=1, work_item=0, name=tuple(b"icm20948".ljust(16, b"\0")), runs=1000,
                                   overruns=0, run_time_last_us=4, run_time_max_us=12, run_time_mean_us=3,
                                   jitter_max_us=900)
    monitor.first_link_status[0] = types.SimpleNamespace(drops=0)  # at the start of the check
    sample = Sample(timestamp=5, value=42)
    sample.FIELDS = ("timestamp", "value")
    for name, message in (("link_status", link), ("work_item_status", worker), ("sample", sample)):
        monitor.topics[(name, 0)] = TopicStats(name, 0)
        monitor.topics[(name, 0)].add(message)

    summary = monitor.summary()
    assert ("mcu_link instance=0 frames_sent=100 bytes_sent=4000 frames_received=5 bytes_received=80 drops=2 "
            "crc_errors=1 malformed=0 lost=3 handshake_state=connected rate_scale=100") in summary
    assert any(line.startswith("work_item name=icm20948 runs=1000 overruns=0") for line in summary)
    assert "the MCU dropped 2 frames on Link 0" in check(monitor, [], 0.05).failures

    view = render(monitor, "udp 127.0.0.1:15600", width=200)
    assert view[0] == "Vehkernel Link  udp 127.0.0.1:15600  handshake ok, protocol 1"
    assert any(line.startswith("mcu link 0") and "drops 2" in line and "lost 3" in line for line in view)
    assert any(line.startswith("sample") and "value=42" in line for line in view)
    assert any(line.split()[:3] == ["icm20948", "1000", "0"] for line in view)
    assert all(len(line) <= 40 for line in render(monitor, "udp", width=40))


def test_attitudes_show_as_roll_pitch_and_yaw_in_degrees():
    half = math.sqrt(0.5)
    assert euler_degrees((1.0, 0.0, 0.0, 0.0)) == pytest.approx((0.0, 0.0, 0.0))
    assert euler_degrees((half, half, 0.0, 0.0)) == pytest.approx((90.0, 0.0, 0.0))
    assert euler_degrees((half, 0.0, 0.0, -half)) == pytest.approx((0.0, 0.0, -90.0))
    assert euler_degrees((half, 0.0, half, 0.0))[1] == pytest.approx(90.0)  # gimbal lock

    connection = types.SimpleNamespace(send=lambda data: None, receive=lambda timeout: b"", close=lambda: None)
    monitor = LinkMonitor(connection, messages())
    attitude = types.SimpleNamespace(FIELDS=("timestamp", "q", "estimator_id"), timestamp=1,
                                     q=(math.cos(0.25), math.sin(0.25), 0.0, 0.0), estimator_id=1)
    monitor.topics[("vehicle_attitude", 0)] = TopicStats("vehicle_attitude", 0)
    monitor.topics[("vehicle_attitude", 0)].add(attitude)
    assert "attitude name=vehicle_attitude instance=0 roll_deg=28.65 pitch_deg=0.00 yaw_deg=0.00" in monitor.summary()
    view = render(monitor, "udp", width=200)
    assert any(line.startswith("vehicle_attitude") and "roll=28.6° pitch=0.0° yaw=0.0° estimator_id=1" in line
               for line in view)


def test_only_losses_and_drops_after_the_handshake_count():
    """A serial adapter may still hold the end of the previous session, and
    the MCU's drops count from boot: neither is this Link's fault."""
    connection = types.SimpleNamespace(send=lambda data: None, receive=lambda timeout: b"", close=lambda: None)
    monitor = LinkMonitor(connection, messages())
    monitor.reader.stats.lost = 650  # a stale frame, then the current stream
    monitor.reader.stats.crc_errors = 1
    monitor.session.handshake_done = True
    monitor.lost_at_handshake = 650
    monitor.crc_errors_at_handshake = 1
    monitor.first_link_status[0] = types.SimpleNamespace(drops=3)
    status = types.SimpleNamespace(timestamp=1, frames_sent=1, bytes_sent=1, frames_received=1, bytes_received=1,
                                   drops=3, crc_errors=0, malformed=0, lost=0, handshake_state=1)
    monitor.topics[("link_status", 0)] = TopicStats("link_status", 0)
    monitor.topics[("link_status", 0)].add(status)
    assert check(monitor, [], 0.05).failures == []
    assert "lost=0" in monitor.summary()[0]

    monitor.reader.stats.lost += 2
    status.drops = 5
    assert check(monitor, [], 0.05).failures == ["2 frames lost", "the MCU dropped 2 frames on Link 0"]

