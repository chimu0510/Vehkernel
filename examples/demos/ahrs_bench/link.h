/*
 * USART1（Type-C 串口 1）的二進位連結：BENCH_BAUD、DMA 收發，主迴圈輪詢，不使用中斷。
 *   - 傳送：雙緩衝，一個由 DMA 送出時另一個累積新訊框。
 *   - 接收：DMA 環形緩衝，link_poll() 取出完整訊框。
 * 啟用後 USART1 不再是文字主控台，不可再呼叫 printf。
 */
#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINK_MAX_PAYLOAD 320U

void link_init(void);

/* 編碼並排入傳送；緩衝不夠時不送、計入 tx_full 並回傳 false，由呼叫者決定稍後重送或放棄。 */
bool link_send(const void *payload, size_t len);

/* 推進傳送；收到完整且 CRC 正確的訊框時把負載寫入 out 並回傳長度，否則回傳 0。 */
size_t link_poll(uint8_t out[LINK_MAX_PAYLOAD]);

typedef struct {
    uint32_t tx_frames, tx_full, rx_frames, rx_errors;
} link_stats_t;

link_stats_t link_stats(void);

#endif /* LINK_H */
