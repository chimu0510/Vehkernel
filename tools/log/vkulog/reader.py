# SPDX-License-Identifier: GPL-3.0-or-later
"""ULog version 1 file reader (spec 0011, ADR-0019)."""

from __future__ import annotations

import io
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, NamedTuple

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


@dataclass(frozen=True)
class FlagBitsMessage:
    compat_flags: bytes
    incompat_flags: bytes
    appended_offsets: tuple[int, int, int]


class ParameterChange(NamedTuple):
    timestamp_us: int
    name: str
    value: int | float

    @property
    def timestamp(self) -> int:
        return self.timestamp_us


@dataclass(frozen=True)
class FormatField:
    type_name: str
    array_size: int  # 0 for scalar
    name: str


@dataclass(frozen=True)
class MessageFormat:
    name: str
    fields: tuple[FormatField, ...]
    raw: str

    _TYPE_MAP = {
        "uint8_t": "B",
        "int8_t": "b",
        "uint16_t": "H",
        "int16_t": "h",
        "uint32_t": "I",
        "int32_t": "i",
        "uint64_t": "Q",
        "int64_t": "q",
        "float": "f",
        "double": "d",
        "bool": "?",
        "char": "s",
    }

    @classmethod
    def parse(cls, format_str: str) -> "MessageFormat":
        parts = format_str.split(":", 1)
        name = parts[0]
        fields = []
        if len(parts) > 1:
            for field_str in parts[1].split(";"):
                field_str = field_str.strip()
                if not field_str:
                    continue
                type_part, _, field_name = field_str.partition(" ")
                field_name = field_name.strip()
                if "[" in type_part and type_part.endswith("]"):
                    base_type, _, size_str = type_part[:-1].partition("[")
                    array_size = int(size_str)
                    type_name = base_type
                else:
                    array_size = 0
                    type_name = type_part
                fields.append(FormatField(type_name, array_size, field_name))
        return cls(name=name, fields=tuple(fields), raw=format_str)

    def struct_format(self) -> str:
        parts = ["<"]
        for f in self.fields:
            code = self._TYPE_MAP.get(f.type_name)
            if code is None:
                raise ValueError(f"Unknown field type {f.type_name!r} in format {self.name}")
            if f.array_size > 0:
                if f.type_name == "char":
                    parts.append(f"{f.array_size}s")
                else:
                    parts.append(f"{f.array_size}{code}")
            else:
                parts.append(code)
        return "".join(parts)

    def decode(self, data: bytes | DataMessage) -> dict[str, Any]:
        """Decode packed message bytes into a dict mapping field name to value or tuple."""
        raw_bytes = data.data if isinstance(data, DataMessage) else data
        fmt_str = self.struct_format()
        expected_size = struct.calcsize(fmt_str)
        if len(raw_bytes) < expected_size:
            raise ValueError(
                f"Data length {len(raw_bytes)} is less than expected {expected_size} for format {self.name}"
            )
        unpacked = struct.unpack(fmt_str, raw_bytes[:expected_size])
        result: dict[str, Any] = {}
        idx = 0
        for f in self.fields:
            if f.array_size == 0:
                result[f.name] = unpacked[idx]
                idx += 1
            elif f.type_name == "char":
                result[f.name] = unpacked[idx]
                idx += 1
            else:
                result[f.name] = tuple(unpacked[idx : idx + f.array_size])
                idx += f.array_size
        return result


@dataclass(frozen=True)
class AddLoggedMessage:
    multi_id: int
    msg_id: int
    message_name: str


@dataclass(frozen=True)
class DataMessage:
    msg_id: int
    data: bytes
    timestamp_us: int  # first uint64 in data


@dataclass(frozen=True)
class DropoutMessage:
    duration_ms: int


@dataclass(frozen=True)
class SyncMessage:
    pass


class ULogReader:
    """Reads ULog version 1 files containing B, I, F, A, P, D, O and S messages."""

    def __init__(self, source: BinaryIO | str | Path | bytes):
        if isinstance(source, bytes):
            self._file: BinaryIO = io.BytesIO(source)
            self._close_on_exit = False
        elif isinstance(source, (str, Path)):
            self._file = open(source, "rb")
            self._close_on_exit = True
        else:
            self._file = source
            self._close_on_exit = False

        self.version: int = 0
        self.start_timestamp_us: int = 0
        self.flag_bits: FlagBitsMessage | None = None
        self.formats: dict[str, MessageFormat] = {}
        self.info: dict[str, Any] = {}
        self.parameters: dict[str, int | float] = {}
        self.parameter_changes: list[ParameterChange] = []
        self.subscriptions: dict[int, AddLoggedMessage] = {}
        self.records: list[Any] = []
        self.data_messages: list[DataMessage] = []
        self.dropouts: list[DropoutMessage] = []
        self.sync_count: int = 0

        self._read()

    def _read(self) -> None:
        header = self._file.read(16)
        if len(header) < 16:
            raise ValueError("ULog file too short (< 16 bytes)")
        if header[:7] != HEADER_BYTES:
            raise ValueError(f"Invalid ULog magic: {header[:7]!r}")
        self.version = header[7]
        (self.start_timestamp_us,) = struct.unpack("<Q", header[8:16])

        in_data_section = False
        last_data_timestamp_us = 0

        while True:
            msg_hdr = self._file.read(3)
            if not msg_hdr:
                break
            if len(msg_hdr) < 3:
                break
            msg_size, msg_type = struct.unpack("<HB", msg_hdr)
            payload = self._file.read(msg_size)
            if len(payload) < msg_size:
                break

            if msg_type == MSG_TYPE_FLAG_BITS:
                compat = payload[0:8]
                incompat = payload[8:16]
                offsets = struct.unpack("<3Q", payload[16:40]) if len(payload) >= 40 else (0, 0, 0)
                fb = FlagBitsMessage(compat, incompat, offsets)
                self.flag_bits = fb
                self.records.append(fb)

            elif msg_type == MSG_TYPE_FORMAT:
                format_str = payload.decode("utf-8", errors="replace")
                fmt = MessageFormat.parse(format_str)
                self.formats[fmt.name] = fmt
                self.records.append(fmt)

            elif msg_type == MSG_TYPE_INFO:
                key, val = self._parse_key_value(payload)
                self.info[key] = val
                self.records.append(("I", key, val))

            elif msg_type == MSG_TYPE_PARAMETER:
                key, val = self._parse_key_value(payload)
                if not in_data_section:
                    self.parameters[key] = val
                    self.records.append(("P", key, val))
                else:
                    change = ParameterChange(last_data_timestamp_us, key, val)
                    self.parameter_changes.append(change)
                    self.records.append(change)

            elif msg_type == MSG_TYPE_ADD_LOGGED_MSG:
                in_data_section = True
                multi_id, msg_id = struct.unpack("<BH", payload[:3])
                name = payload[3:].decode("utf-8", errors="replace")
                sub = AddLoggedMessage(multi_id=multi_id, msg_id=msg_id, message_name=name)
                self.subscriptions[msg_id] = sub
                self.records.append(sub)

            elif msg_type == MSG_TYPE_DATA:
                in_data_section = True
                (msg_id,) = struct.unpack("<H", payload[:2])
                data = payload[2:]
                ts = struct.unpack("<Q", data[:8])[0] if len(data) >= 8 else 0
                if ts > last_data_timestamp_us:
                    last_data_timestamp_us = ts
                dm = DataMessage(msg_id=msg_id, data=data, timestamp_us=ts)
                self.data_messages.append(dm)
                self.records.append(dm)

            elif msg_type == MSG_TYPE_DROPOUT:
                in_data_section = True
                (duration_ms,) = struct.unpack("<H", payload[:2])
                drop = DropoutMessage(duration_ms=duration_ms)
                self.dropouts.append(drop)
                self.records.append(drop)

            elif msg_type == MSG_TYPE_SYNC:
                self.sync_count += 1
                self.records.append(SyncMessage())

    def _parse_key_value(self, payload: bytes) -> tuple[str, Any]:
        key_len = payload[0]
        type_key = payload[1 : 1 + key_len].decode("utf-8", errors="replace")
        type_str, _, key = type_key.partition(" ")
        val_bytes = payload[1 + key_len :]
        if type_str.startswith("char["):
            val: Any = val_bytes.decode("utf-8", errors="replace").rstrip("\x00")
        elif type_str == "int32_t":
            (val,) = struct.unpack("<i", val_bytes)
        elif type_str == "uint32_t":
            (val,) = struct.unpack("<I", val_bytes)
        elif type_str == "int64_t":
            (val,) = struct.unpack("<q", val_bytes)
        elif type_str == "uint64_t":
            (val,) = struct.unpack("<Q", val_bytes)
        elif type_str == "float":
            (val,) = struct.unpack("<f", val_bytes)
        elif type_str == "double":
            (val,) = struct.unpack("<d", val_bytes)
        elif type_str == "bool":
            (val,) = struct.unpack("<?", val_bytes)
        else:
            val = val_bytes
        return key, val

    def decode_message(self, data_msg: DataMessage) -> dict[str, Any]:
        """Decode a DataMessage using its format from this log."""
        sub = self.subscriptions.get(data_msg.msg_id)
        if sub is None:
            raise KeyError(f"Unknown msg_id {data_msg.msg_id}")
        fmt = self.formats.get(sub.message_name)
        if fmt is None:
            raise KeyError(f"Unknown format for message '{sub.message_name}'")
        return fmt.decode(data_msg)

    def close(self) -> None:
        if self._close_on_exit:
            self._file.close()

    def __enter__(self) -> "ULogReader":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
