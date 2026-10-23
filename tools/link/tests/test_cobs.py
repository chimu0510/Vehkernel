# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for COBS encoding and decoding (ADR-0002)."""

import pytest
from vklink.cobs import cobs_decode, cobs_encode, cobs_max_encoded_size

KNOWN_VECTORS = [
    (b"", b"\x01"),
    (b"\x00", b"\x01\x01"),
    (b"\x00\x00", b"\x01\x01\x01"),
    (b"\x00\x11\x00", b"\x01\x02\x11\x01"),
    (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"),
    (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
]


@pytest.mark.parametrize("unencoded,encoded", KNOWN_VECTORS)
def test_known_vectors(unencoded: bytes, encoded: bytes):
    assert cobs_encode(unencoded) == encoded
    assert cobs_decode(encoded) == unencoded


def test_long_blocks():
    # 254 non-zero bytes fill exactly one block of 255 bytes (code 0xFF + 254 bytes).
    in254 = b"\xaa" * 254
    enc254 = cobs_encode(in254)
    assert len(enc254) == 255
    assert enc254[0] == 0xFF
    assert cobs_decode(enc254) == in254

    # 255 non-zero bytes overflow into a second block.
    in255 = b"\xbb" * 255
    enc255 = cobs_encode(in255)
    assert len(enc255) == 257
    assert enc255[0] == 0xFF
    assert enc255[255] == 0x02
    assert enc255[256] == 0xBB
    assert cobs_decode(enc255) == in255


def test_cobs_max_encoded_size():
    assert cobs_max_encoded_size(0) == 1
    assert cobs_max_encoded_size(254) == 256
    assert cobs_max_encoded_size(255) == 257


@pytest.mark.parametrize(
    "corrupted",
    [
        b"\x00\x01",      # Zero code byte
        b"\x03\x00\x01",  # Zero byte inside data block
        b"\x05\x01",      # Code byte points past end of data
    ],
)
def test_corrupted_encoding_raises(corrupted: bytes):
    with pytest.raises(ValueError):
        cobs_decode(corrupted)
