# SPDX-License-Identifier: GPL-3.0-or-later
"""ULog version 1 file writer (spec 0011, ADR-0019)."""

from __future__ import annotations

import struct
from pathlib import Path
from typing import BinaryIO

from .constants import (
    FILE_VERSION,
    HEADER_BYTES,
    MSG_TYPE_ADD_LOGGED_MSG,
    MSG_TYPE_DATA,
    MSG_TYPE_DROPOUT,
    MSG_TYPE_FLAG_BITS,
    MSG_TYPE_FORMAT,
    MSG_TYPE_INFO,
    MSG_TYPE_PARAMETER,
    MSG_TYPE_SYNC,
    SYNC_BYTES,
)


class ULogWriter:
    """Writes ULog version 1 files containing B, I, F, A, P, D, O and S messages."""

    def __init__(self, target: BinaryIO | str | Path, start_timestamp_us: int = 0):
        if isinstance(target, (str, Path)):
            self._file: BinaryIO = open(target, "wb")
            self._close_on_exit = True
        else:
            self._file = target
            self._close_on_exit = False
        self._start_timestamp_us = start_timestamp_us
        self._write_header(start_timestamp_us)

    def _write_header(self, start_timestamp_us: int) -> None:
        header = HEADER_BYTES + struct.pack("B", FILE_VERSION) + struct.pack("<Q", start_timestamp_us)
        self._file.write(header)
        # The spec's file format puts a Flag Bits message (B) right after the header.
        self.write_flag_bits()

    def write_flag_bits(self, compat_flags: bytes = b"\x00" * 8, incompat_flags: bytes = b"\x00" * 8,
                        appended_offsets: tuple[int, int, int] = (0, 0, 0)) -> None:
        """Write a 'B' flag bits message as the first message of the definitions section."""
        payload = compat_flags[:8].ljust(8, b"\x00") + incompat_flags[:8].ljust(8, b"\x00") + struct.pack("<3Q", *appended_offsets)
        header = struct.pack("<HB", len(payload), MSG_TYPE_FLAG_BITS)
        self._file.write(header + payload)

    def write_format(self, format_string: str) -> None:
        """Write an 'F' format message defining a message schema."""
        data = format_string.encode("utf-8")
        header = struct.pack("<HB", len(data), MSG_TYPE_FORMAT)
        self._file.write(header + data)

    def write_info(self, key: str, value: str | int | float | bytes, type_name: str | None = None) -> None:
        """Write an 'I' message with information/metadata."""
        payload = self._encode_key_value(key, value, type_name)
        header = struct.pack("<HB", len(payload), MSG_TYPE_INFO)
        self._file.write(header + payload)

    def write_parameter(self, name: str, value: int | float) -> None:
        """Write a 'P' message with an initial or changed parameter value."""
        if isinstance(value, float):
            type_name = "float"
            val_bytes = struct.pack("<f", value)
        elif isinstance(value, int):
            type_name = "int32_t"
            val_bytes = struct.pack("<i", value)
        else:
            raise TypeError(f"Unsupported parameter value type: {type(value)}")
        type_key = f"{type_name} {name}".encode("utf-8")
        payload = struct.pack("<B", len(type_key)) + type_key + val_bytes
        header = struct.pack("<HB", len(payload), MSG_TYPE_PARAMETER)
        self._file.write(header + payload)

    def write_add_logged(self, msg_id: int, message_name: str, multi_id: int = 0) -> None:
        """Write an 'A' message mapping msg_id to message_name and multi_id (instance)."""
        name_bytes = message_name.encode("utf-8")
        payload = struct.pack("<BH", multi_id, msg_id) + name_bytes
        header = struct.pack("<HB", len(payload), MSG_TYPE_ADD_LOGGED_MSG)
        self._file.write(header + payload)

    def write_data(self, msg_id: int, data: bytes) -> None:
        """Write a 'D' message containing packed data for msg_id."""
        payload = struct.pack("<H", msg_id) + data
        header = struct.pack("<HB", len(payload), MSG_TYPE_DATA)
        self._file.write(header + payload)

    def write_dropout(self, duration_ms: int) -> None:
        """Write an 'O' message indicating a dropout of duration_ms."""
        payload = struct.pack("<H", duration_ms)
        header = struct.pack("<HB", len(payload), MSG_TYPE_DROPOUT)
        self._file.write(header + payload)

    def write_sync(self) -> None:
        """Write an 'S' sync message."""
        header = struct.pack("<HB", len(SYNC_BYTES), MSG_TYPE_SYNC)
        self._file.write(header + SYNC_BYTES)

    def _encode_key_value(self, key: str, value: str | int | float | bytes, type_name: str | None) -> bytes:
        if isinstance(value, str):
            val_bytes = value.encode("utf-8")
            actual_type = f"char[{len(val_bytes)}]" if type_name is None else type_name
        elif isinstance(value, bytes):
            val_bytes = value
            actual_type = f"uint8_t[{len(val_bytes)}]" if type_name is None else type_name
        elif isinstance(value, float):
            val_bytes = struct.pack("<f", value)
            actual_type = "float" if type_name is None else type_name
        elif isinstance(value, int):
            val_bytes = struct.pack("<i", value)
            actual_type = "int32_t" if type_name is None else type_name
        else:
            raise TypeError(f"Unsupported info value type: {type(value)}")
        type_key = f"{actual_type} {key}".encode("utf-8")
        return struct.pack("<B", len(type_key)) + type_key + val_bytes

    def close(self) -> None:
        if self._close_on_exit:
            self._file.close()

    def __enter__(self) -> "ULogWriter":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
