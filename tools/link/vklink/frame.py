# SPDX-License-Identifier: GPL-3.0-or-later
"""Link frames of the custom protocol (ADR-0002).

Wire format:
  [COBS encoded: version(1) | type(1) | sequence(2) | body(N) | crc16(2)] 0x00
"""

import enum
import struct
from dataclasses import dataclass

from .cobs import cobs_decode, cobs_encode, cobs_max_encoded_size
from .crc16 import crc16

# Version 2 added parameter access (ADR-0012), version 3 mode, arming and
# emergency stop requests (ADR-0005), version 4 reboot requests (ADR-0023),
# version 5 logging (spec 0011, ADR-0019); the MCU offers only its current version.
PROTOCOL_VERSION = 5
# A version only adds to the one before, so the host offers every version from
# the oldest it speaks (ADR-0002). Version 1's hello_reply lacks the parameter
# fields that version 2 appended.
OLDEST_PROTOCOL_VERSION = 2

HEADER_SIZE = 4
CRC_SIZE = 2
MAX_MESSAGE_SIZE = 240
MAX_BODY_SIZE = 3 + MAX_MESSAGE_SIZE
MAX_FRAME_SIZE = HEADER_SIZE + MAX_BODY_SIZE + CRC_SIZE
MAX_WIRE_SIZE = cobs_max_encoded_size(MAX_FRAME_SIZE) + 1


class LogStartResult(enum.IntEnum):
    OK = 0
    ARMED = 1
    UNKNOWN_PROFILE = 2
    OVER_BUDGET = 3
    ALREADY_LOGGING = 4


class FrameType(enum.IntEnum):
    TOPIC_DATA = 0x01
    HELLO = 0x10
    HELLO_REPLY = 0x11
    TOPIC_INFO = 0x12
    TIME_SYNC_REQUEST = 0x20
    TIME_SYNC_REPLY = 0x21
    PARAM_LIST_REQUEST = 0x30
    PARAM_GET = 0x31
    PARAM_SET = 0x32
    PARAM_VALUE = 0x33
    PARAM_REJECT = 0x34
    MODE_REQUEST = 0x40
    MODE_REPLY = 0x41
    MODE_REJECT = 0x42
    ARMING_REQUEST = 0x43
    ARMING_REPLY = 0x44
    ARMING_REJECT = 0x45
    ESTOP_REQUEST = 0x46
    ESTOP_REPLY = 0x47
    BRINGUP_RUN = 0x48
    BRINGUP_REJECT = 0x49
    BRINGUP_RESULT = 0x4A
    REBOOT_REQUEST = 0x4B
    REBOOT_REPLY = 0x4C
    REBOOT_REJECT = 0x4D
    LOG_START = 0x50
    LOG_INFO = 0x51
    LOG_STOP = 0x52
    LOG_DROPOUT = 0x53
    # SITL only: cuts or restores the simulated actuator power.
    SIM_ACTUATOR_POWER = 0x70
    # SITL only: presses or releases the simulated user key.
    SIM_USER_KEY = 0x71
    SIM_GAMEPAD = 0x72


# The version each frame the host sends first appeared in; the host refuses to
# send one the negotiated version does not have (ADR-0002).
HOST_FRAME_VERSIONS = {
    FrameType.TOPIC_DATA: 1,
    FrameType.HELLO: 1,
    FrameType.TIME_SYNC_REQUEST: 1,
    FrameType.PARAM_LIST_REQUEST: 2,
    FrameType.PARAM_GET: 2,
    FrameType.PARAM_SET: 2,
    FrameType.MODE_REQUEST: 3,
    FrameType.ARMING_REQUEST: 3,
    FrameType.ESTOP_REQUEST: 3,
    FrameType.BRINGUP_RUN: 3,
    FrameType.SIM_ACTUATOR_POWER: 3,
    FrameType.SIM_USER_KEY: 3,
    FrameType.REBOOT_REQUEST: 4,
    FrameType.SIM_GAMEPAD: 4,
    FrameType.LOG_START: 5,
    FrameType.LOG_STOP: 5,
}


class ProtocolVersionError(ValueError):
    """A frame the negotiated protocol version does not have."""

    def __init__(self, frame_type: FrameType, needed: int, negotiated: int):
        super().__init__(f"{frame_type.name.lower()} needs protocol version {needed}, "
                         f"but the MCU speaks version {negotiated}; update its firmware")
        self.frame_type = frame_type
        self.needed = needed
        self.negotiated = negotiated


@dataclass(frozen=True)
class LogInfo:
    status: LogStartResult
    board_id: int
    version_major: int
    version_minor: int
    git_hash: int
    bandwidth: float
    board_name: str

    @classmethod
    def parse(cls, body: bytes) -> "LogInfo | None":
        if len(body) < 14:
            return None
        raw_status, board_id, ver_maj, ver_min, git_hash, bandwidth, name_len = struct.unpack("<BHBBIfB", body[:14])
        if len(body) < 14 + name_len:
            return None
        name = body[14:14 + name_len].decode("utf-8", errors="replace")
        return cls(LogStartResult(raw_status), board_id, ver_maj, ver_min, git_hash, bandwidth, name)


@dataclass(frozen=True)
class LogDropout:
    topic_id: int
    instance: int
    count: int

    @classmethod
    def parse(cls, body: bytes) -> "LogDropout | None":
        if len(body) < 5:
            return None
        topic_id, instance, count = struct.unpack("<HBH", body[:5])
        return cls(topic_id, instance, count)


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
        # Written in each frame's header: the negotiated version once there is one.
        self.version = PROTOCOL_VERSION

    @property
    def next_sequence(self) -> int:
        return self._sequence

    def write(self, frame_type: FrameType, body: bytes) -> bytes:
        if len(body) > MAX_BODY_SIZE:
            raise ValueError(f"Body size {len(body)} exceeds maximum {MAX_BODY_SIZE}")
        header = struct.pack("<BBH", self.version, int(frame_type), self._sequence)
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
