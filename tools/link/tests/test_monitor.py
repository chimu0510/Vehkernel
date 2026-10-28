# SPDX-License-Identifier: GPL-3.0-or-later
"""The Link monitor against a fake MCU over UDP on the loopback (spec 0003)."""

import math
import socket
import struct
import threading
import types

import pytest

from vklink import PROTOCOL_VERSION, FrameReader, McuSession, TopicDescription
from vklink.monitor import (LinkMonitor, RateExpectation, TopicStats, UdpConnection, WorkItemOverruns, check,
                            euler_degrees, work_item_overruns_line)
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

    def __init__(self, skip_sequence: bool = False, params=None, param_set_hash: int = 0, control=None,
                 topics=(), protocol_version: int = PROTOCOL_VERSION):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.002)
        self.port = self.socket.getsockname()[1]
        self.session = McuSession([TopicDescription(0, 1, SAMPLE_HASH, "sample"), *topics], params=params,
                                  param_set_hash=param_set_hash, control=control, protocol_version=protocol_version)
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
    assert summary[0].startswith(f"link handshake=ok version={PROTOCOL_VERSION} mismatches=0")
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



def work_item_reports(*reports, rates=None):
    """WorkItemOverruns fed one report a second, each a {name: overruns} map;
    `rates` gives each work item's runs a second (none: every one 200)."""
    tracker = WorkItemOverruns()
    for second, report in enumerate(reports):
        for index, (name, overruns) in enumerate(report.items()):
            runs = second * (rates or {}).get(name, 200)
            tracker.record(types.SimpleNamespace(timestamp=second * 1_000_000, work_item=index,
                                                 name=tuple(name.encode().ljust(16, b"\0")), overruns=overruns,
                                                 runs=runs))
    return tracker


def test_work_item_overruns_set_aside_a_stall_of_the_whole_process():
    # Start-up overruns, then one interval where the four periodic items of six overran together.
    tracker = work_item_reports({"imu": 5, "ctrl": 5, "act": 5, "link": 6, "est": 0, "timing": 0},
                                {"imu": 5, "ctrl": 5, "act": 5, "link": 6, "est": 0, "timing": 0},
                                {"imu": 8, "ctrl": 8, "act": 9, "link": 9, "est": 0, "timing": 0},
                                {"imu": 8, "ctrl": 8, "act": 9, "link": 9, "est": 0, "timing": 0})
    assert tracker.judge() == ({"imu": 0, "ctrl": 0, "act": 0, "link": 0, "est": 0, "timing": 0}, 1)


def test_work_item_overruns_judge_a_stall_among_the_work_items_at_the_base_rate():
    # Four periodic items at 200 Hz overran together; four slow ones (10 and
    # 1 Hz) could not have. Among all ten that is fewer than half, but among
    # the six at the base rate, two of them triggered, it is most: a stall.
    slow = {"menu": 0, "display": 0, "storage": 0, "timing": 0}
    rates = {"menu": 100, "display": 20, "storage": 20, "timing": 1}
    tracker = work_item_reports({"imu": 0, "ctrl": 0, "act": 0, "link": 0, "est": 0, "odom": 0, **slow},
                                {"imu": 1, "ctrl": 1, "act": 1, "link": 1, "est": 0, "odom": 0, **slow},
                                rates=rates)
    assert tracker.judge() == (dict.fromkeys(["imu", "ctrl", "act", "link", "est", "odom", *slow], 0), 1)
    # One base-rate item alone still counts.
    tracker = work_item_reports({"imu": 0, "ctrl": 0, "act": 0, "link": 0, "est": 0, "odom": 0, **slow},
                                {"imu": 0, "ctrl": 1, "act": 0, "link": 0, "est": 0, "odom": 0, **slow},
                                rates=rates)
    assert tracker.judge()[0]["ctrl"] == 1


def test_work_item_overruns_count_a_few_work_items_overrunning_alone():
    tracker = work_item_reports({"imu": 0, "ctrl": 0, "act": 0, "link": 0, "est": 0, "timing": 0},
                                {"imu": 0, "ctrl": 2, "act": 1, "link": 0, "est": 0, "timing": 0},
                                {"imu": 0, "ctrl": 2, "act": 1, "link": 0, "est": 0, "timing": 0})
    assert tracker.judge() == ({"imu": 0, "ctrl": 2, "act": 1, "link": 0, "est": 0, "timing": 0}, 0)


def test_work_item_overruns_line_gives_the_stalls_and_the_real_overruns():
    assert work_item_overruns_line(None) == "work_item_overruns judged=0"
    assert work_item_overruns_line(({"imu": 0, "link": 0}, 1)) == "work_item_overruns judged=1 stalls=1 real=0"
    assert work_item_overruns_line(({"link": 2, "imu": 0, "act": 1}, 0)) == \
        "work_item_overruns judged=1 stalls=0 real=3 overran=act:1,link:2"


def test_work_item_overruns_need_two_reports_with_every_work_item():
    assert WorkItemOverruns().judge() is None
    # The second report lacks a work item, so there is no interval to judge.
    assert work_item_reports({"imu": 0, "ctrl": 0}, {"imu": 1}).judge() is None
