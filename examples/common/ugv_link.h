/*
 * C50C 與樹莓派 5 之間的高速 DMA 串口通訊鏈路 (ADR-0003)
 * USART1 (Type-C 串口 1): TX DMA2_Stream7_Ch4, RX DMA2_Stream2_Ch4 (循環緩衝)
 */
#ifndef UGV_LINK_H
#define UGV_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ugv_protocol.h"

#define UGV_DEFAULT_BAUD 115200U

typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t rx_errors;
    uint32_t tx_full;
} ugv_link_stats_t;

void ugv_link_init(uint32_t baud);

/**
 * 送出結構體封包（自動進行 CRC16 與 COBS 分幀編碼，寫入 DMA 發送佇列）
 */
bool ugv_link_send(const void *payload, size_t len);

/**
 * 輪詢接收封包：從 DMA 循環緩衝讀取並還原完整的 COBS 訊框與 CRC16 校驗
 * @param out_msg: 解碼後的封包儲存位置
 * @return 成功解碼之負載長度 (bytes)；若無完整訊框則回傳 0
 */
size_t ugv_link_poll(ugv_msg_payload_t *out_msg);

ugv_link_stats_t ugv_link_get_stats(void);

#endif /* UGV_LINK_H */
