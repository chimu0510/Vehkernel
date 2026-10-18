"""CRC-16/CCITT-FALSE (ADR-0002).

Polynomial 0x1021, initial value 0xFFFF, no reflection, no final XOR.
"""


def crc16(data: bytes) -> int:
    """Calculates CRC-16/CCITT-FALSE over `data`."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
