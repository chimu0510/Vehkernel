/*
 * 姿態估測（Mahony 互補濾波，四元數）：陀螺儀積分，加速度計修正 roll/pitch。
 *   - 沒有磁力計，yaw 只靠陀螺儀積分，長時間會漂移（ADR-0002：絕對航向交給 ROS2 端 EKF）。
 *   - 加速度大小偏離 1 g 超過 AHRS_ACCEL_GATE_G 時（加減速、碰撞），不採用加速度修正。
 *   - 零偏由呼叫者先扣掉（見 gyro_cal.h），這裡不估零偏（Ki = 0）。
 * 座標：IMU 本體座標；元件面朝上時 z 朝上，角度依 ROS REP-103：
 *   roll 繞 x、pitch 繞 y、yaw 繞 z，皆以右手定則為正（yaw 由上往下看逆時針為正）。
 */
#ifndef AHRS_H
#define AHRS_H

#include <stdbool.h>

#define AHRS_KP            1.0f    /* 加速度修正增益，1/s；越大越快貼近重力方向、越容易受加減速干擾 */
#define AHRS_ACCEL_GATE_G  0.15f

/* 以加速度（重力方向）決定初始 roll/pitch，yaw = 0。 */
void ahrs_reset(const float accel_g[3]);

/*
 * gyro_dps 已扣零偏；hold 為 true 時（靜止）不積分陀螺儀，只做加速度修正，
 * 角度因此不會隨殘餘零偏漂移。
 */
void ahrs_update(const float gyro_dps[3], const float accel_g[3], float dt, bool hold);

/* 回傳 ZYX 尤拉角（度）；yaw 範圍 -180..+180。 */
void ahrs_euler(float *roll_deg, float *pitch_deg, float *yaw_deg);

/* 最近一次是否採用了加速度修正。 */
bool ahrs_accel_used(void);

#endif /* AHRS_H */
