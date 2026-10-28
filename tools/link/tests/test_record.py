# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for host log recording and dropout handling (spec 0011)."""

import os
import socket
import struct
import subprocess
import sys
import threading
from pathlib import Path

import pytest
import pyulog
import vkulog
from vklink import FrameType, TopicDescription
from vklink.frame import FrameReader, FrameWriter
from vklink.monitor import LinkMonitor, UdpConnection, load_messages
from vklink.recorder import record
from vklink.session import McuSession

MSG_DIR = os.environ.get("VK_MSG_PYTHON_DIR")
if not MSG_DIR:
    cand = Path(__file__).resolve().parents[3] / "build/test/generated/schema/python"
    if cand.exists():
        MSG_DIR = str(cand)


class FakeLoggingMcu:
    """Answers handshake, time sync, log_start, sends topic messages, drops frames, and sends log_dropout."""

    def __init__(self, topics=None, sender_fn=None):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.005)
        self.port = self.socket.getsockname()[1]
        self.topics = topics if topics is not None else [
            TopicDescription(10, 1, 0x1234, "imu_sample"),
            TopicDescription(5, 1, 0x5678, "body_velocity_measurement"),
        ]
        self.session = McuSession(self.topics)
        self.reader = FrameReader()
        self.writer = FrameWriter()
        self.peer = None
        self.now_us = 1_000_000
        self.sent = 0
        self.sender_fn = sender_fn
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
                self.peer = sender
                for frame in self.reader.feed(data):
                    for reply in self.session.handle(frame, self.now_us):
                        self.socket.sendto(reply, sender)
            except TimeoutError:
                pass
            if not self.peer:
                continue

            if self.sender_fn is not None:
                self.sender_fn(self)
                continue

            self.now_us += 10_000
            # Send an imu_sample (topic_id 10)
            sample_data = struct.pack("<Qfffffff", self.now_us, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0, 25.0)
            wire = self.session.send_topic(10, 0, sample_data, self.now_us)
            if wire is None:
                continue
            self.sent += 1

            # 1. Transport drop: frame 5 is numbered by McuSession but not sent over the wire
            if self.sent == 5:
                continue

            self.socket.sendto(wire, self.peer)

            # 2. Log dropout: at frame 10, send a log_dropout message
            if self.sent == 10:
                dropout_body = struct.pack("<HBH", 10, 0, 4)  # 4 missed samples on imu_sample
                drop_wire = self.session.writer.write(FrameType.LOG_DROPOUT, dropout_body)
                self.socket.sendto(drop_wire, self.peer)


def test_record_detects_frame_gaps_and_log_dropouts(tmp_path):
    if not MSG_DIR or not Path(MSG_DIR).exists():
        pytest.skip("VK_MSG_PYTHON_DIR not set or generated messages missing")

    messages = load_messages(MSG_DIR)
    log_file = tmp_path / "test_dropouts.ulg"
    with FakeLoggingMcu() as mcu:
        conn = UdpConnection("127.0.0.1", mcu.port)
        monitor = LinkMonitor(conn, messages)
        assert monitor.handshake(timeout_s=5.0)
        ret = record(
            monitor,
            "estimator_replay",
            log_file,
            duration=0.3,
            msg_dir=MSG_DIR,
        )
        assert ret == 0

    assert log_file.exists()
    reader = vkulog.ULogReader(log_file)
    assert reader.start_timestamp_us > 0
    assert "sys_name" in reader.info
    assert len(reader.dropouts) >= 2
    # Check that dropouts have non-zero duration
    durations = [d.duration_ms for d in reader.dropouts]
    assert all(d > 0 for d in durations)


def test_record_logs_all_instances_of_topic(tmp_path):
    if not MSG_DIR or not Path(MSG_DIR).exists():
        pytest.skip("VK_MSG_PYTHON_DIR not set or generated messages missing")

    messages = load_messages(MSG_DIR)
    log_file = tmp_path / "test_multi_instance.ulg"

    # link_status is an all-instances topic in estimator_replay (instances = 2)
    topics = [
        TopicDescription(11, 2, 0x5D65F66B5AA01822, "link_status"),
    ]

    def send_link_status_instances(mcu: FakeLoggingMcu) -> None:
        mcu.now_us += 20_000
        # link_status wire format: uint64 timestamp, 8*uint32, 2*uint8, uint32
        payload_0 = struct.pack("<QIIIIIIIIBBI", mcu.now_us, 1, 100, 1, 100, 0, 0, 0, 0, 1, 1, 0)
        wire_0 = mcu.session.send_topic(11, 0, payload_0, mcu.now_us)
        if wire_0:
            mcu.socket.sendto(wire_0, mcu.peer)

        payload_1 = struct.pack("<QIIIIIIIIBBI", mcu.now_us, 2, 200, 2, 200, 0, 0, 0, 0, 1, 1, 0)
        wire_1 = mcu.session.send_topic(11, 1, payload_1, mcu.now_us)
        if wire_1:
            mcu.socket.sendto(wire_1, mcu.peer)

    with FakeLoggingMcu(topics=topics, sender_fn=send_link_status_instances) as mcu:
        conn = UdpConnection("127.0.0.1", mcu.port)
        monitor = LinkMonitor(conn, messages)
        assert monitor.handshake(timeout_s=5.0)
        ret = record(
            monitor,
            "estimator_replay",
            log_file,
            duration=0.3,
            msg_dir=MSG_DIR,
        )
        assert ret == 0

    assert log_file.exists()

    # 1. Verify through vkulog
    reader = vkulog.ULogReader(log_file)
    subs = {s.msg_id: (s.message_name, s.multi_id) for s in reader.subscriptions.values()}
    assert ("link_status", 0) in subs.values()
    assert ("link_status", 1) in subs.values()

    id_inst0 = [k for k, v in subs.items() if v == ("link_status", 0)][0]
    id_inst1 = [k for k, v in subs.items() if v == ("link_status", 1)][0]
    assert id_inst0 != id_inst1

    logged_ids = {m.msg_id for m in reader.data_messages}
    assert id_inst0 in logged_ids
    assert id_inst1 in logged_ids

    # 2. Verify through pyulog
    ulog = pyulog.ULog(str(log_file))
    data_by_topic = {(d.name, d.multi_id): d for d in ulog.data_list}
    assert ("link_status", 0) in data_by_topic
    assert ("link_status", 1) in data_by_topic

    ds0 = data_by_topic[("link_status", 0)]
    ds1 = data_by_topic[("link_status", 1)]
    assert ds0.multi_id == 0
    assert ds1.multi_id == 1
    assert len(ds0.data["timestamp"]) > 0
    assert len(ds1.data["timestamp"]) > 0
    assert ds0.data["frames_sent"][0] == 1
    assert ds1.data["frames_sent"][0] == 2


def test_cli_record_fails_on_an_older_mcu(tmp_path):
    if not MSG_DIR or not Path(MSG_DIR).exists():
        pytest.skip("VK_MSG_PYTHON_DIR not set or generated messages missing")
    from test_monitor import FakeMcu
    from test_params import MONITOR

    # Logging came with protocol version 5; the host refuses to ask a version 4 MCU for it (ADR-0002).
    with FakeMcu(protocol_version=4) as mcu:
        result = subprocess.run(
            [sys.executable, str(MONITOR), "--udp", f"127.0.0.1:{mcu.port}", "--msg-dir", MSG_DIR,
             "--handshake-timeout", "2", "record", str(tmp_path / "old.ulg"), "--profile", "estimator_replay",
             "--duration", "1"],
            capture_output=True, text=True, timeout=30, check=False)
    assert result.returncode == 1, result.stderr
    assert "FAIL: log_start needs protocol version 5, but the MCU speaks version 4" in result.stderr
    assert not (tmp_path / "old.ulg").exists()
