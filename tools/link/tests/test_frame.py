"""Tests for FrameWriter and FrameReader (ADR-0002)."""

import struct
import pytest
from vklink.cobs import cobs_encode
from vklink.crc16 import crc16
from vklink.frame import (
    HEADER_SIZE,
    MAX_BODY_SIZE,
    MAX_FRAME_SIZE,
    MAX_WIRE_SIZE,
    PROTOCOL_VERSION,
    FrameReader,
    FrameType,
    FrameWriter,
)


def test_frame_writer_produces_valid_wire_frame():
    writer = FrameWriter(start_sequence=0)
    body = b"\x10\x20\x30"
    wire = writer.write(FrameType.HELLO, body)

    assert wire.endswith(b"\x00")
    assert writer.next_sequence == 1

    reader = FrameReader()
    frames = reader.feed(wire)
    assert len(frames) == 1
    assert frames[0].version == PROTOCOL_VERSION
    assert frames[0].type == FrameType.HELLO
    assert frames[0].sequence == 0
    assert frames[0].body == body


def test_frame_writer_rejects_oversized_body():
    writer = FrameWriter()
    big_body = b"\x00" * (MAX_BODY_SIZE + 1)
    with pytest.raises(ValueError):
        writer.write(FrameType.TOPIC_DATA, big_body)


def test_frame_reader_handles_concatenated_frames():
    writer = FrameWriter()
    stream = bytearray()
    for i in range(3):
        stream.extend(writer.write(FrameType.TOPIC_DATA, bytes([i])))

    reader = FrameReader()
    frames = reader.feed(bytes(stream))
    assert len(frames) == 3
    assert [f.sequence for f in frames] == [0, 1, 2]
    assert [f.body for f in frames] == [b"\x00", b"\x01", b"\x02"]
    assert reader.stats.frames == 3


def test_frame_reader_ignores_consecutive_delimiters():
    reader = FrameReader()
    frames = reader.feed(b"\x00\x00\x00\x00")
    assert not frames
    assert reader.stats.frames == 0
    assert reader.stats.malformed == 0


def test_frame_reader_catches_crc_error():
    writer = FrameWriter()
    wire = bytearray(writer.write(FrameType.TOPIC_DATA, b"\x01\x02\x03"))
    # Corrupt payload byte
    wire[len(wire) // 2] ^= 0xFF

    reader = FrameReader()
    frames = reader.feed(bytes(wire))
    assert not frames
    assert reader.stats.crc_errors == 1


def test_frame_reader_catches_malformed_input():
    reader = FrameReader()
    # Truncated frame (ends with 0x00 early)
    frames = reader.feed(b"\x02\x01\x00")
    assert not frames
    assert reader.stats.malformed == 1


def test_frame_reader_catches_oversized_input():
    reader = FrameReader()
    too_long = b"\x01" * (MAX_WIRE_SIZE + 10) + b"\x00"
    frames = reader.feed(too_long)
    assert not frames
    assert reader.stats.malformed == 1


def test_frame_reader_recovers_at_next_delimiter():
    writer = FrameWriter()
    valid_wire = writer.write(FrameType.HELLO, b"\x42\x43")

    stream = b"\xFF\xFE\xFD\x00" + valid_wire

    reader = FrameReader()
    frames = reader.feed(stream)
    assert len(frames) == 1
    assert frames[0].type == FrameType.HELLO
    assert frames[0].body == b"\x42\x43"
    assert reader.stats.frames == 1
    assert reader.stats.malformed == 1


def test_frame_reader_tracks_lost_frames_and_restart():
    reader = FrameReader()

    def make_raw_frame(seq: int) -> bytes:
        plain = struct.pack("<BBH", PROTOCOL_VERSION, int(FrameType.TOPIC_DATA), seq)
        plain += struct.pack("<H", crc16(plain))
        return cobs_encode(plain) + b"\x00"

    reader.feed(make_raw_frame(0))
    assert reader.stats.lost == 0

    # Sequence jumps 0 -> 4: 1, 2, 3 lost
    reader.feed(make_raw_frame(4))
    assert reader.stats.lost == 3

    # Peer restarts backwards to 0: must not count as lost
    reader.feed(make_raw_frame(0))
    assert reader.stats.lost == 3
