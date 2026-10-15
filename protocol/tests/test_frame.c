/**
 * @file test_frame.c
 * @brief Unit tests for Layer 2 Framing: COBS (Consistent Overhead Byte Stuffing)
 *        and CRC16-CCITT integrity verification (ADR-0003).
 *
 * This test suite is completely decoupled from high-level message schemas.
 * It operates strictly on raw byte payloads.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ugv_protocol/frame.h"

static void test_crc16_ccitt_standard_vector(void)
{
    printf("[TEST] CRC16-CCITT standard test vector...\n");
    // Standard ASCII sequence "123456789" with poly 0x1021 and init 0xFFFF gives 0x29B1
    const uint8_t data[] = "123456789";
    const uint16_t crc = crc16_ccitt(data, 9, 0xFFFFU);
    assert(crc == 0x29B1U);
    printf("  CRC16('123456789') = 0x%04X (expected 0x29B1) [PASS]\n", crc);
}

static void test_frame_encode_delimiter(void)
{
    printf("[TEST] Frame delimiter and zero-elimination properties...\n");
    const uint8_t payload[] = {0x01, 0x00, 0x02, 0x00, 0x03};
    uint8_t frame_buf[FRAME_MAX_ENCODED(sizeof(payload))];

    size_t frame_len = frame_encode(payload, sizeof(payload), frame_buf);
    assert(frame_len > sizeof(payload));

    // Must terminate with 0x00 delimiter
    assert(frame_buf[frame_len - 1] == 0x00);

    // Frame body MUST NOT contain any 0x00 bytes before the delimiter
    for (size_t i = 0; i < frame_len - 1; ++i) {
        assert(frame_buf[i] != 0x00);
    }
    printf("  Delimiter & COBS non-zero payload guarantee [PASS]\n");
}

static void test_frame_roundtrip_patterns(void)
{
    printf("[TEST] Frame encode/decode roundtrip patterns...\n");

    // Pattern 1: All zeros (triggers maximum COBS code splitting)
    {
        const uint8_t zeros[16] = {0};
        uint8_t frame[FRAME_MAX_ENCODED(sizeof(zeros))];
        size_t flen = frame_encode(zeros, sizeof(zeros), frame);

        uint8_t decoded[sizeof(frame)];
        memcpy(decoded, frame, flen - 1);
        int dlen = frame_decode(decoded, flen - 1);
        assert(dlen == (int)sizeof(zeros));
        assert(memcmp(decoded, zeros, sizeof(zeros)) == 0);
    }

    // Pattern 2: No zeros (continuous block up to 100 bytes)
    {
        uint8_t no_zeros[100];
        for (size_t i = 0; i < sizeof(no_zeros); ++i) {
            no_zeros[i] = (uint8_t)((i % 254) + 1);
        }
        uint8_t frame[FRAME_MAX_ENCODED(sizeof(no_zeros))];
        size_t flen = frame_encode(no_zeros, sizeof(no_zeros), frame);

        uint8_t decoded[sizeof(frame)];
        memcpy(decoded, frame, flen - 1);
        int dlen = frame_decode(decoded, flen - 1);
        assert(dlen == (int)sizeof(no_zeros));
        assert(memcmp(decoded, no_zeros, sizeof(no_zeros)) == 0);
    }

    // Pattern 3: Block boundary crossing (255 non-zero bytes, exceeds single 254-byte COBS chunk)
    {
        uint8_t large[255];
        memset(large, 0xAA, sizeof(large));
        uint8_t frame[FRAME_MAX_ENCODED(sizeof(large))];
        size_t flen = frame_encode(large, sizeof(large), frame);

        uint8_t decoded[sizeof(frame)];
        memcpy(decoded, frame, flen - 1);
        int dlen = frame_decode(decoded, flen - 1);
        assert(dlen == (int)sizeof(large));
        assert(memcmp(decoded, large, sizeof(large)) == 0);
    }

    printf("  All-zeros, non-zeros, and chunk boundary crossing [PASS]\n");
}

static void test_frame_error_detection(void)
{
    printf("[TEST] Frame error rejection (corruption & truncation)...\n");

    const uint8_t payload[] = "Robotics educational test message";
    const size_t plen = sizeof(payload);
    uint8_t frame[FRAME_MAX_ENCODED(plen)];
    size_t flen = frame_encode(payload, plen, frame);

    // Case 1: Corrupt payload byte
    {
        uint8_t corrupted[sizeof(frame)];
        memcpy(corrupted, frame, flen - 1);
        corrupted[5] ^= 0xFF; // Bit-flip in payload
        int res = frame_decode(corrupted, flen - 1);
        assert(res == -1); // Must reject due to CRC mismatch
    }

    // Case 2: Corrupt CRC byte
    {
        uint8_t corrupted[sizeof(frame)];
        memcpy(corrupted, frame, flen - 1);
        corrupted[flen - 2] ^= 0x01; // Corrupt CRC
        int res = frame_decode(corrupted, flen - 1);
        assert(res == -1);
    }

    // Case 3: Truncated frame
    {
        uint8_t truncated[sizeof(frame)];
        memcpy(truncated, frame, flen - 1);
        int res = frame_decode(truncated, 3); // Too short to hold CRC
        assert(res == -1);
    }

    // Case 4: Invalid COBS pointer leading past buffer end
    {
        uint8_t invalid_cobs[] = {0xFE, 0x01, 0x02}; // Code says next 0 is at +254, but buffer is only 3 bytes
        int res = frame_decode(invalid_cobs, sizeof(invalid_cobs));
        assert(res == -1);
    }

    printf("  Corrupted payload, corrupted CRC, truncated, and invalid COBS rejected [PASS]\n");
}

int main(void)
{
    printf("========================================\n");
    printf(" Running Decoupled Framing Tests (Layer 2)\n");
    printf("========================================\n");
    test_crc16_ccitt_standard_vector();
    test_frame_encode_delimiter();
    test_frame_roundtrip_patterns();
    test_frame_error_detection();
    printf("==> ALL FRAMING TESTS PASSED!\n\n");
    return 0;
}
