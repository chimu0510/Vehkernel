/*
 * 四路正交編碼器（腳位見 board.h），定時器編碼器模式、四倍頻。
 * TIM3/TIM4 只有 16 位元，所以以 encoder_poll() 讀取差值累加成 32 位元計數；
 * 輪子以 1 圈/秒轉動時約 36000 計數/秒，每 100 ms 內至少呼叫一次即不會溢位。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef enum {
    ENC_A = 0,
    ENC_B,
    ENC_C,
    ENC_D,
    ENC_COUNT,
} encoder_id_t;

void encoder_init(void);

/* 讀取硬體計數器並累加；回傳前把累計值寫入 counts（可為 NULL）。 */
void encoder_poll(int32_t counts[ENC_COUNT]);

void encoder_reset(void);

const char *encoder_name(encoder_id_t id);

#endif /* ENCODER_H */
