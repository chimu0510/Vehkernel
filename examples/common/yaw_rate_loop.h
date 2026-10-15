/*
 * 偏航角速度 wz 陀螺儀閉環補償控制器 (ADR-0002)
 */
#ifndef YAW_RATE_LOOP_H
#define YAW_RATE_LOOP_H

#include <stdbool.h>
#include "pid.h"

typedef struct {
    pid_controller_t pid;
    bool enabled;
    float max_correction_wz; /* 最大允許修正量 (rad/s) */
} yaw_rate_loop_t;

void yaw_rate_loop_init(yaw_rate_loop_t *loop, float kp, float ki, float kd, float max_correction);

void yaw_rate_loop_reset(yaw_rate_loop_t *loop);

void yaw_rate_loop_enable(yaw_rate_loop_t *loop, bool enable);

/**
 * 計算補償後的車體角速度：
 * @param target_wz 目標角速度 (rad/s)
 * @param measured_gyro_wz 陀螺儀量測之角速度 (rad/s, REP-103: 逆時針為正)
 * @param dt 取樣週期 (s)
 * @return 經過閉環補償修正後的角速度 (rad/s)，直接輸入運動學分配給左右輪
 */
float yaw_rate_loop_update(yaw_rate_loop_t *loop, float target_wz, float measured_gyro_wz, float dt);

#endif /* YAW_RATE_LOOP_H */
