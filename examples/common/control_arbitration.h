/*
 * 控制來源仲裁與斜坡失效保護狀態機 (ADR-0004)
 * 固定優先序: SBUS (1) > USB 手柄 (2) > Raspberry Pi 5 (3)
 */
#ifndef CONTROL_ARBITRATION_H
#define CONTROL_ARBITRATION_H

#include <stdbool.h>
#include <stdint.h>
#include "ugv_protocol.h"  /* 引用共用協議定義之 ugv_control_source_t, ugv_operating_mode_t */

/* 來源輸入指令結構 */
typedef struct {
    float vx;            /* 目標線速度 (m/s) */
    float wz;            /* 目標角速度 (rad/s) */
    bool active;         /* 該來源當前是否處於活躍操作中 (例如撥桿不在自主檔位/手柄推動) */
    bool healthy;        /* 該來源連線是否正常 (未超時、無訊號遺失) */
    uint32_t last_rx_ms; /* 最後接收到有效指令的開機毫秒時間戳 */
} control_input_t;

typedef struct {
    float max_linear_accel;   /* 最大線加速度 (m/s^2) */
    float max_linear_decel;   /* 最大減速度 (m/s^2, 用於失效保護斜坡停車) */
    float max_angular_accel;  /* 最大角加速度 (rad/s^2) */
    float max_angular_decel;  /* 最大角減速度 (rad/s^2) */
    uint32_t sbus_timeout_ms; /* SBUS 逾時門檻 (e.g. 100 ms) */
    uint32_t ros2_timeout_ms; /* ROS2 心跳/指令逾時門檻 (e.g. 250 ms) */
} arbitration_config_t;

typedef struct {
    arbitration_config_t config;

    control_input_t sbus;
    control_input_t gamepad;
    control_input_t ros2;

    ugv_control_source_t active_source;
    ugv_operating_mode_t current_mode;

    /* 斜坡平滑後的當前指令輸出 */
    float cmd_vx;
    float cmd_wz;

    bool failsafe_active;
    bool estop_active;
    bool low_battery_active;
} arbitration_state_t;

void arbitration_init(arbitration_state_t *arb, const arbitration_config_t *cfg);

/**
 * 餵入 SBUS 來源狀態
 * @param active: 是否有主動搖桿控制 (非自主擋位)
 * @param healthy: 訊框未遺失且無 failsafe 旗標
 */
void arbitration_feed_sbus(arbitration_state_t *arb, float vx, float wz, bool active, bool healthy, uint32_t now_ms);

/**
 * 餵入 USB 手柄來源狀態
 */
void arbitration_feed_gamepad(arbitration_state_t *arb, float vx, float wz, bool active, bool healthy, uint32_t now_ms);

/**
 * 餵入 ROS 2 上位機指令
 */
void arbitration_feed_ros2(arbitration_state_t *arb, float vx, float wz, uint32_t now_ms);

/**
 * 外部緊急開關與低電量狀態更新
 */
void arbitration_set_estop(arbitration_state_t *arb, bool estop);
void arbitration_set_low_battery(arbitration_state_t *arb, bool low_battery);

/**
 * 定頻更新狀態機仲裁與平滑減速斜坡
 * @param dt: 取樣週期 (s)
 * @param now_ms: 當前毫秒時間戳
 * @param out_vx: 仲裁後當前線速度指令 (m/s)
 * @param out_wz: 仲裁後當前角速度指令 (rad/s)
 */
void arbitration_update(arbitration_state_t *arb, float dt, uint32_t now_ms, float *out_vx, float *out_wz);

#endif /* CONTROL_ARBITRATION_H */
