# SPDX-License-Identifier: GPL-3.0-or-later
"""Link protocol package for host-side tools (ADR-0002)."""

from .cobs import cobs_decode, cobs_encode, cobs_max_encoded_size
from .crc16 import crc16
from .frame import (
    HEADER_SIZE,
    MAX_BODY_SIZE,
    MAX_FRAME_SIZE,
    MAX_MESSAGE_SIZE,
    MAX_WIRE_SIZE,
    PROTOCOL_VERSION,
    Frame,
    FrameReader,
    FrameType,
    FrameWriter,
    ReaderStats,
)
from .session import (
    REJECT_REASONS,
    BringupReply,
    ControlReply,
    HostSession,
    McuControl,
    McuParam,
    McuSession,
    ParamReject,
    ParamType,
    ParamValue,
    TimeSyncResult,
    TopicDescription,
)

__all__ = [
    "cobs_encode",
    "cobs_decode",
    "cobs_max_encoded_size",
    "crc16",
    "PROTOCOL_VERSION",
    "HEADER_SIZE",
    "MAX_MESSAGE_SIZE",
    "MAX_BODY_SIZE",
    "MAX_FRAME_SIZE",
    "MAX_WIRE_SIZE",
    "FrameType",
    "Frame",
    "ReaderStats",
    "FrameWriter",
    "FrameReader",
    "TopicDescription",
    "TimeSyncResult",
    "HostSession",
    "McuSession",
    "McuParam",
    "ParamType",
    "ParamValue",
    "ParamReject",
    "REJECT_REASONS",
    "ControlReply",
    "McuControl",
    "BringupReply",
]
