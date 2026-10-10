/*
 * 姿態估測（乘性誤差狀態 EKF，MEKF）：陀螺儀積分預測，加速度計與靜止時的零角速度修正。
 *
 * 狀態：名義狀態為四元數 q（本體到世界）與陀螺儀零偏 b；濾波的是 6 維誤差狀態
 *       [δθ（本體座標的小角度誤差）, δb]，共變異數 P 為 6×6。每次修正後把誤差併回名義狀態。
 * 預測：ω = ω_量測 − b，q ← q ⊗ exp(ω·dt)；P ← F P Fᵀ + Q（角度隨機遊走、零偏隨機遊走）。
 * 量測：
 *   - 加速度：量到的重力方向。R 隨 |a| 偏離 1 g 放大（加減速時權重自動降低），
 *     偏離超過 EKF_ACCEL_REJECT_G 時不採用。修正 roll/pitch 與 x、y 軸零偏。
 *   - 零角速度（ZUPT）：靜止時以「真實角速度 = 0」為量測，零偏因此三軸都可觀測，
 *     包括加速度計看不到的 z 軸。
 * 靜止偵測：陀螺儀原始讀值的變異數小（不需要先知道零偏）、扣掉估計零偏後的角速度
 *   小於 EKF_STILL_DPS、|a| 接近 1 g，且持續 EKF_STILL_SECONDS。
 *   比 EKF_STILL_DPS 慢的等速轉動會被當成零偏；主韌體應再加上編碼器的靜止條件。
 * 沒有磁力計：yaw 不可觀測，σ_yaw 會隨時間成長（ADR-0002：絕對航向交給 ROS2 端）。
 * 角度依 ROS REP-103，IMU 本體座標，元件面朝上時 z 朝上。
 */
#ifndef EKF_AHRS_H
#define EKF_AHRS_H

#include <stdbool.h>
#include <stdint.h>

/* 雜訊參數（ICM-20948，±500 dps、DLPF 51 Hz，已留餘量給震動）。 */
#define EKF_GYRO_NOISE_DPS     0.05f    /* 角度隨機遊走，dps/√Hz */
#define EKF_BIAS_WALK_DPS      0.002f   /* 零偏隨機遊走，dps/s/√Hz */
#define EKF_ACCEL_NOISE        0.02f    /* 重力方向量測雜訊（單位向量） */
#define EKF_ACCEL_ADAPT        5.0f     /* 每偏離 1 g 的 1%，雜訊增加 0.05 */
#define EKF_ACCEL_REJECT_G     0.3f
#define EKF_ZUPT_NOISE_DPS     0.1f     /* 靜止時角速度量測雜訊 */
#define EKF_INIT_ANGLE_DEG     5.0f     /* 初始 roll/pitch 不確定度 */
#define EKF_INIT_BIAS_DPS      1.0f     /* 初始零偏不確定度 */

#define EKF_STILL_VAR_DPS      0.3f     /* 原始角速度的標準差上限 */
#define EKF_STILL_DPS          3.0f
#define EKF_STILL_G            0.05f
#define EKF_STILL_SECONDS      0.5f

typedef struct {
    float q[4];
    float b[3];              /* 零偏，rad/s */
    float P[6][6];
    /* 靜止偵測 */
    float mean[3], var[3];   /* 原始角速度的指數移動平均與變異數，(rad/s)、(rad/s)² */
    uint32_t quiet_n;
    bool still;
    /* 統計 */
    uint32_t accel_updates, accel_rejects, zupt_updates;
} ekf_ahrs_t;

/* 以加速度（重力方向）決定初始 roll/pitch，yaw = 0，零偏 = 0。 */
void ekf_ahrs_init(ekf_ahrs_t *f, const float accel_g[3]);

/* gyro_dps 為原始讀值（不要先扣零偏）；dt 為取樣間隔，秒。 */
void ekf_ahrs_step(ekf_ahrs_t *f, const float gyro_dps[3], const float accel_g[3], float dt);

/* ZYX 尤拉角，度；yaw 範圍 -180..+180。 */
void ekf_ahrs_euler(const ekf_ahrs_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg);

void ekf_ahrs_bias_dps(const ekf_ahrs_t *f, float bias_dps[3]);

/* 1σ 不確定度：姿態誤差（本體座標，度；姿態接近水平時約等於 roll/pitch/yaw）與零偏（dps）。 */
void ekf_ahrs_sigma(const ekf_ahrs_t *f, float angle_deg[3], float bias_dps[3]);

#endif /* EKF_AHRS_H */
