"""Tests for CRC-16/CCITT-FALSE (ADR-0002)."""

from vklink.crc16 import crc16


def test_standard_vector():
    # Standard ASCII test vector "123456789" -> 0x29B1.
    assert crc16(b"123456789") == 0x29B1


def test_empty_input():
    assert crc16(b"") == 0xFFFF
