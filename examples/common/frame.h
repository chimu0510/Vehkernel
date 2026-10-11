/*
 * 串口訊框：負載後接 CRC16（CCITT-FALSE，little endian），整段以 COBS 編碼，訊框以 0x00 結尾。
 * COBS 保證編碼後的資料不含 0x00，所以接收端遇到 0x00 就知道一個訊框結束，掉位元組或從中途
 * 開始接收時最多丟一個訊框就能重新同步。純計算，不依賴 HAL，主機端也用同一套規則。
 */
#ifndef FRAME_H
#define FRAME_H

#include <stddef.h>
#include <stdint.h>

/* 編碼後最大長度：負載 + CRC 2 + COBS 每 254 bytes 一個 overhead + 起頭 1 + 結尾 0x00。 */
#define FRAME_MAX_ENCODED(n) ((n) + 2U + ((n) + 2U) / 254U + 2U)

uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc);

/* 把 payload 編成一個完整訊框（含結尾 0x00）；out 至少 FRAME_MAX_ENCODED(len)。回傳長度。 */
size_t frame_encode(const uint8_t *payload, size_t len, uint8_t *out);

/*
 * 解碼一個訊框（不含結尾 0x00），結果寫回 buf（原地解碼）。
 * 回傳負載長度（已去掉 CRC）；COBS 格式錯誤或 CRC 不符回傳 -1。
 */
int frame_decode(uint8_t *buf, size_t len);

#endif /* FRAME_H */
