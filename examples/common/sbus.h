/*
 * SBUS 航模接收機解碼 (CN7, UART5_RX PD2)
 */
#ifndef SBUS_H
#define SBUS_H

#include <stdbool.h>
#include <stdint.h>

#define SBUS_CHANNELS 16U

typedef struct {
    uint16_t ch[SBUS_CHANNELS];  /* 11-bit 通道值 (約 172 ~ 1811, 中位點 ~992) */
    bool frame_lost;
    bool failsafe;
    uint32_t last_rx_ms;
} sbus_state_t;

void sbus_init(void);

/**
 * 讀取當前 SBUS 狀態
 * @return true 若接收機正常且未逾時 (100ms 內有更新)
 */
bool sbus_read(sbus_state_t *out_state);

/**
 * 將 SBUS 搖桿通道轉換為車體目標速度 (Vx, wz)
 * 通道 1 為轉向 (ch[0])，通道 2 為前進油門 (ch[1])
 * @param max_vx: 最大前進速度 (m/s)
 * @param max_wz: 最大旋轉角速度 (rad/s)
 * @param out_active: 使用者是否有推動搖桿 (不在死區內)
 */
bool sbus_get_velocity(const sbus_state_t *state, float max_vx, float max_wz,
                       float *out_vx, float *out_wz, bool *out_active);

#endif /* SBUS_H */
