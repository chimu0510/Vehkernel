/*
 * 姿態估測（Mahony 互補濾波，四元數）：陀螺儀積分，加速度計修正 roll/pitch。
 *   - 誤差 e = â × v（量到與估計的重力方向的外積），角速度修正 ω ← ω − b + Kp·e。
 *   - Ki > 0 時以積分項估零偏：b ← b − Ki·e·dt。e 只在重力的垂直平面內，所以水平時 z 軸零偏看不到。
 *   - 沒有磁力計，yaw 只靠陀螺儀積分，長時間會漂移（ADR-0002：絕對航向交給 ROS2 端 EKF）。
 *   - 加速度大小偏離 1 g 超過 accel_gate_g 時（加減速、碰撞），不採用加速度修正。
 * 座標：IMU 本體座標；元件面朝上時 z 朝上，角度依 ROS REP-103：
 *   roll 繞 x、pitch 繞 y、yaw 繞 z，皆以右手定則為正（yaw 由上往下看逆時針為正）。
 */
#ifndef AHRS_H
#define AHRS_H

#include <stdbool.h>

typedef struct {
    float kp;            /* 加速度修正增益，1/s；越大越快貼近重力方向、越容易受加減速干擾 */
    float ki;            /* 零偏積分增益，1/s²；0 表示不估零偏（由呼叫者先扣掉） */
    float accel_gate_g;
} ahrs_params_t;

#define AHRS_DEFAULT_PARAMS { .kp = 1.0f, .ki = 0.0f, .accel_gate_g = 0.15f }

typedef struct {
    ahrs_params_t p;
    float q[4];          /* 本體到世界座標的四元數 */
    float b[3];          /* 積分項估的零偏，rad/s */
    float e[3];          /* 最近一次的誤差 â × v */
    bool accel_used;     /* 最近一次是否採用了加速度修正 */
} ahrs_t;

/* 以加速度（重力方向）決定初始 roll/pitch，yaw = 0，零偏歸零；p 為 NULL 時用預設參數。 */
void ahrs_reset(ahrs_t *f, const ahrs_params_t *p, const float accel_g[3]);

/*
 * gyro_dps 為角速度（Ki = 0 時應已扣零偏）；hold 為 true 時（靜止）不積分陀螺儀、
 * 不更新零偏，只做加速度修正，角度因此不會隨殘餘零偏漂移。
 */
void ahrs_update(ahrs_t *f, const float gyro_dps[3], const float accel_g[3], float dt, bool hold);

/* 回傳 ZYX 尤拉角（度）；yaw 範圍 -180..+180。 */
void ahrs_euler(const ahrs_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif /* AHRS_H */
