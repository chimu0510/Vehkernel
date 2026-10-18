"""Link frames of the custom protocol (ADR-0002).

Wire format:
  [COBS encoded: version(1) | type(1) | sequence(2) | body(N) | crc16(2)] 0x00
"""

import enum
import struct
from dataclasses import dataclass

from .cobs import cobs_decode, cobs_encode, cobs_max_encoded_size
from .crc16 import crc16

PROTOCOL_VERSION = 1

HEADER_SIZE = 4
CRC_SIZE = 2
MAX_MESSAGE_SIZE = 240
MAX_BODY_SIZE = 3 + MAX_MESSAGE_SIZE
MAX_FRAME_SIZE = HEADER_SIZE + MAX_BODY_SIZE + CRC_SIZE
MAX_WIRE_SIZE = cobs_max_encoded_size(MAX_FRAME_SIZE) + 1


class FrameType(enum.IntEnum):
    TOPIC_DATA = 0x01
    HELLO = 0x10
    HELLO_REPLY = 0x11
    TOPIC_INFO = 0x12
    TIME_SYNC_REQUEST = 0x20
    TIME_SYNC_REPLY = 0x21


@dataclass(frozen=True)
class Frame:
    version: int
    type: FrameType
    sequence: int
    body: bytes


@dataclass
class ReaderStats:
    frames: int = 0
    crc_errors: int = 0
    malformed: int = 0
    lost: int = 0


class FrameWriter:
    """Encodes frames for the wire, advancing sequence numbers."""

    def __init__(self, start_sequence: int = 0):
        self._sequence = start_sequence

    @property
    def next_sequence(self) -> int:
        return self._sequence

    def write(self, frame_type: FrameType, body: bytes) -> bytes:
        if len(body) > MAX_BODY_SIZE:
            raise ValueError(f"Body size {len(body)} exceeds maximum {MAX_BODY_SIZE}")
        header = struct.pack("<BBH", PROTOCOL_VERSION, int(frame_type), self._sequence)
        plain = header + body
        plain += struct.pack("<H", crc16(plain))
        wire = cobs_encode(plain) + b"\x00"
        self._sequence = (self._sequence + 1) & 0xFFFF
        return wire


class FrameReader:
    """Streams bytes into frames, resynchronizing after corruptions."""

    def __init__(self):
        self.stats = ReaderStats()
        self._buffer = bytearray()
        self._overflow = False
        self._have_sequence = False
        self._expected_sequence = 0

    def feed(self, data: bytes) -> list[Frame]:
        frames = []
        for byte in data:
            if byte != 0:
                if len(self._buffer) < MAX_WIRE_SIZE:
                    self._buffer.append(byte)
                else:
                    self._overflow = True
                continue

            raw = bytes(self._buffer)
            overflow = self._overflow
            self._buffer.clear()
            self._overflow = False

            if not raw:
                continue

            if overflow:
                self.stats.malformed += 1
                continue

            try:
                plain = cobs_decode(raw)
            except ValueError:
                self.stats.malformed += 1
                continue

            if len(plain) < HEADER_SIZE + CRC_SIZE:
                self.stats.malformed += 1
                continue

            payload_crc = struct.unpack("<H", plain[-CRC_SIZE:])[0]
            if crc16(plain[:-CRC_SIZE]) != payload_crc:
                self.stats.crc_errors += 1
                continue

            version, raw_type, sequence = struct.unpack("<BBH", plain[:HEADER_SIZE])
            try:
                frame_type = FrameType(raw_type)
            except ValueError:
                self.stats.malformed += 1
                continue

            body = plain[HEADER_SIZE:-CRC_SIZE]

            if self._have_sequence:
                gap = (sequence - self._expected_sequence) & 0xFFFF
                if gap < 0x8000:
                    self.stats.lost += gap
            self._have_sequence = True
            self._expected_sequence = (sequence + 1) & 0xFFFF
            self.stats.frames += 1

            frames.append(Frame(version=version, type=frame_type, sequence=sequence, body=body))

        return frames
