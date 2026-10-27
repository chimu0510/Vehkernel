# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for host log recording and dropout handling (spec 0011)."""

import os
import socket
import struct
import threading
from pathlib import Path

import pytest
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

    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", 0))
        self.socket.settimeout(0.005)
        self.port = self.socket.getsockname()[1]
        self.topics = [
            TopicDescription(10, 1, 0x1234, "imu_sample"),
            TopicDescription(5, 1, 0x5678, "body_velocity_measurement"),
        ]
        self.session = McuSession(self.topics)
        self.reader = FrameReader()
        self.writer = FrameWriter()
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
                self.peer = sender
                for frame in self.reader.feed(data):
                    for reply in self.session.handle(frame, self.now_us):
                        self.socket.sendto(reply, sender)
            except TimeoutError:
                pass
            if not self.peer:
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
