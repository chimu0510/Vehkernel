# SPDX-License-Identifier: GPL-3.0-or-later
"""Consistent Overhead Byte Stuffing (COBS, ADR-0002).

Removes every zero byte from a payload so a zero can delimit frames on the wire.
"""


def cobs_max_encoded_size(size: int) -> int:
    """Largest COBS encoding of `size` bytes, without the delimiter."""
    return size + size // 254 + 1


def cobs_encode(data: bytes) -> bytes:
    """Encodes `data` into COBS without trailing delimiter (ADR-0002)."""
    out = bytearray()
    code_at = 0
    out.append(1)
    code = 1
    for i, byte in enumerate(data):
        if byte != 0:
            out.append(byte)
            code += 1
        if byte == 0 or code == 0xFF:
            out[code_at] = code
            code = 1
            code_at = len(out)
            if byte == 0 or i + 1 < len(data):
                out.append(1)
            else:
                return bytes(out)
    out[code_at] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Decodes COBS `data` (without delimiter). Raises ValueError if malformed."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            raise ValueError("Invalid COBS code byte")
        for _ in range(1, code):
            byte = data[i]
            i += 1
            if byte == 0:
                raise ValueError("Zero byte in COBS data block")
            out.append(byte)
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)
