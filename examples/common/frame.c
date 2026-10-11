#include "frame.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* COBS：每段以「到下一個 0x00 的距離」開頭，段長上限 254。 */
static size_t cobs_put(uint8_t *out, size_t *code_pos, size_t pos, uint8_t *code, uint8_t byte)
{
    if (byte != 0U) {
        out[pos++] = byte;
        (*code)++;
    }
    if (byte == 0U || *code == 0xFFU) {
        out[*code_pos] = *code;
        *code_pos = pos++;
        *code = 1U;
    }
    return pos;
}

size_t frame_encode(const uint8_t *payload, size_t len, uint8_t *out)
{
    const uint16_t crc = crc16_ccitt(payload, len, 0xFFFFU);
    const uint8_t tail[2] = {(uint8_t)(crc & 0xFFU), (uint8_t)(crc >> 8)};
    size_t code_pos = 0, pos = 1;
    uint8_t code = 1U;

    for (size_t i = 0; i < len; i++) {
        pos = cobs_put(out, &code_pos, pos, &code, payload[i]);
    }
    for (size_t i = 0; i < 2; i++) {
        pos = cobs_put(out, &code_pos, pos, &code, tail[i]);
    }
    out[code_pos] = code;
    out[pos++] = 0U;
    return pos;
}

int frame_decode(uint8_t *buf, size_t len)
{
    size_t in = 0, out = 0;

    while (in < len) {
        const uint8_t code = buf[in++];
        if (code == 0U || in + code - 1U > len) {
            return -1;
        }
        for (uint8_t i = 1; i < code; i++) {
            buf[out++] = buf[in++];
        }
        if (code != 0xFFU && in < len) {
            buf[out++] = 0U;
        }
    }
    if (out < 2U) {
        return -1;
    }
    const uint16_t crc = (uint16_t)(buf[out - 2U] | (buf[out - 1U] << 8));
    if (crc16_ccitt(buf, out - 2U, 0xFFFFU) != crc) {
        return -1;
    }
    return (int)(out - 2U);
}
