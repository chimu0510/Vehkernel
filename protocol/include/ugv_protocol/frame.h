/*
 * ugv_protocol frame framing and CRC16 routines.
 * COBS (Consistent Overhead Byte Stuffing) + CRC16-CCITT (0x1021).
 * Compatible with C99/C11 and C++14/C++17/C++20.
 */
#ifndef UGV_PROTOCOL_FRAME_H
#define UGV_PROTOCOL_FRAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum encoded size: payload + CRC 2 + COBS overhead (1 byte per 254) + 1 head + 1 tail 0x00 */
#define FRAME_MAX_ENCODED(n) ((n) + 2U + ((n) + 2U) / 254U + 2U)

/**
 * Calculate CRC16-CCITT (poly 0x1021, init 0xFFFF).
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc);

/**
 * Encodes payload into a complete frame (including CRC16 and tail 0x00 delimiter).
 * Output buffer must be at least FRAME_MAX_ENCODED(len) bytes.
 * Returns total encoded length including the trailing 0x00.
 */
size_t frame_encode(const uint8_t *payload, size_t len, uint8_t *out);

/**
 * Decodes a frame (excluding the trailing 0x00 delimiter) in-place in buf.
 * Returns payload length (with CRC stripped), or -1 if CRC check or COBS format fails.
 */
int frame_decode(uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* UGV_PROTOCOL_FRAME_H */
