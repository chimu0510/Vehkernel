# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for Link session (ADR-0002).

Covers handshake negotiation, schema hash comparisons, time synchronization,
peer timeout and topic data gating.
"""

from vklink.frame import FrameReader, PROTOCOL_VERSION
from vklink.session import HostSession, McuSession, TopicDescription


def test_handshake_picks_highest_shared_version():
    topics = [
        TopicDescription(id=1, instances=1, schema_hash=0x1122334455667788, name="imu_sample"),
        TopicDescription(id=2, instances=2, schema_hash=0x99AABBCCDDEEFF00, name="status"),
    ]
    mcu = McuSession(topics=topics)
    host = HostSession(
        local_schema={
            "imu_sample": 0x1122334455667788,
            "status": 0x99AABBCCDDEEFF00,
        },
        supported_versions=list(range(1, PROTOCOL_VERSION + 1)),
    )

    # Host initiates handshake
    hello_wire = host.make_hello()
    reader_mcu = FrameReader()
    mcu_in_frames = reader_mcu.feed(hello_wire)
    assert len(mcu_in_frames) == 1

    # MCU processes Hello and replies with HelloReply + 2 TopicInfo
    replies = mcu.handle(mcu_in_frames[0], now_us=1000)
    assert len(replies) == 3

    reader_host = FrameReader()
    for reply_wire in replies:
        for f in reader_host.feed(reply_wire):
            host.handle_frame(f)

    assert host.handshake_done
    assert host.version == PROTOCOL_VERSION
    assert len(host.peer_topics) == 2
    assert host.peer_topics[1].name == "imu_sample"
    assert host.peer_topics[2].instances == 2
    assert host.mismatched_topics == []


def test_handshake_refuses_incompatible_peer():
    mcu = McuSession()
    host = HostSession(supported_versions=[1])  # before parameter access; the MCU offers only its version

    hello_wire = host.make_hello()
    mcu_frames = FrameReader().feed(hello_wire)
    replies = mcu.handle(mcu_frames[0], now_us=1000)
    assert len(replies) == 1  # Only HelloReply(chosen=0)

    host_frames = FrameReader().feed(replies[0])
    host.handle_frame(host_frames[0])

    assert not host.handshake_done
    assert host.version == 0


def test_handshake_reports_mismatched_schema_hashes():
    topics = [
        TopicDescription(id=1, instances=1, schema_hash=0x1111, name="topic_ok"),
        TopicDescription(id=2, instances=1, schema_hash=0x2222, name="topic_mismatch"),
    ]
    mcu = McuSession(topics=topics)
    host = HostSession(
        local_schema={
            "topic_ok": 0x1111,
            "topic_mismatch": 0x9999,  # Different hash
        }
    )

    mcu_frames = FrameReader().feed(host.make_hello())
    replies = mcu.handle(mcu_frames[0], now_us=1000)

    reader = FrameReader()
    for wire in replies:
        for f in reader.feed(wire):
            host.handle_frame(f)

    assert host.handshake_done
    assert host.mismatched_topics == ["topic_mismatch"]


def test_time_sync_computation():
    mcu = McuSession()
    host = HostSession()

    # Handshake first
    mcu_frames = FrameReader().feed(host.make_hello())
    for reply in mcu.handle(mcu_frames[0], now_us=1000):
        for f in FrameReader().feed(reply):
            host.handle_frame(f)

    # Host sends TimeSyncRequest at host time 10_000_000 us
    req_wire = host.make_time_sync(host_send_time_us=10_000_000)
    req_frames = FrameReader().feed(req_wire)

    # MCU receives at MCU time 500_000 us and replies
    replies = mcu.handle(req_frames[0], now_us=500_000)
    assert len(replies) == 1

    # Host receives reply at host time 10_020_000 us (20 ms round trip)
    host_frames = FrameReader().feed(replies[0])
    result = host.parse_time_sync(host_frames[0], host_recv_us=10_020_000)

    assert result.host_send_us == 10_000_000
    assert result.mcu_recv_us == 500_000
    assert result.mcu_send_us == 500_000
    assert result.host_recv_us == 10_020_000
    assert result.rtt_us == 20_000
    # Clock offset = ((500_000 - 10_000_000) + (500_000 - 10_020_000)) // 2 = -9_510_000
    assert result.clock_offset_us == -9_510_000


def test_topic_data_gated_by_handshake_and_timeout():
    timeout_us = 1_000_000
    mcu = McuSession(peer_timeout_us=timeout_us)
    host = HostSession()

    sample = b"\x01\x02\x03\x04"

    # 1. Topic data cannot be sent before handshake
    assert mcu.send_topic(1, 0, sample, now_us=100) is None

    # 2. Handshake at now_us = 100
    mcu_frames = FrameReader().feed(host.make_hello())
    mcu.handle(mcu_frames[0], now_us=100)
    assert mcu.connected(now_us=100)

    # 3. Topic data is sent while connected
    wire = mcu.send_topic(1, 0, sample, now_us=500)
    assert wire is not None
    assert wire.endswith(b"\x00")

    # 4. Peer timeout expires after 1s of silence
    assert not mcu.connected(now_us=100 + timeout_us + 1)
    assert mcu.send_topic(1, 0, sample, now_us=100 + timeout_us + 1) is None

    # 5. Peer sends time sync, keeping session alive
    sync_req = FrameReader().feed(host.make_time_sync(12345))[0]
    mcu.handle(sync_req, now_us=2_000_000)

    assert mcu.connected(now_us=2_000_000)
    assert mcu.send_topic(1, 0, sample, now_us=2_500_000) is not None
