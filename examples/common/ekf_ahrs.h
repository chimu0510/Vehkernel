/*
 * 姿態估測（乘性誤差狀態 EKF，MEKF）：陀螺儀積分預測，加速度計與靜止時的零角速度修正。
 *
 * 狀態：名義狀態為四元數 q（本體到世界）與陀螺儀零偏 b；濾波的是 6 維誤差狀態
 *       [δθ（本體座標的小角度誤差）, δb]，共變異數 P 為 6×6。每次修正後把誤差併回名義狀態。
 * 預測：ω = ω_量測 − b，q ← q ⊗ exp(ω·dt)；P ← F P Fᵀ + Q（角度隨機遊走、零偏隨機遊走）。
 * 量測：
 *   - 加速度：量到的重力方向。R 隨 |a| 偏離 1 g 放大（加減速時權重自動降低），
 *     偏離超過 accel_reject_g 時不採用。修正 roll/pitch 與 x、y 軸零偏。
 *   - 零角速度（ZUPT）：靜止時以「真實角速度 = 0」為量測，零偏因此三軸都可觀測，
 *     包括加速度計看不到的 z 軸。
 * 靜止偵測：陀螺儀原始讀值的變異數小（不需要先知道零偏）、扣掉估計零偏後的角速度
 *   小於 still_dps、|a| 接近 1 g，且持續 still_s。
 *   比 still_dps 慢的等速轉動會被當成零偏；主韌體應再加上編碼器的靜止條件。
 * 沒有磁力計：yaw 不可觀測，σ_yaw 會隨時間成長（ADR-0002：絕對航向交給 ROS2 端）。
 * 角度依 ROS REP-103，IMU 本體座標，元件面朝上時 z 朝上。
 */
#ifndef EKF_AHRS_H
#define EKF_AHRS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 參數（執行期可調，ahrs_bench 經串口修改）。雜訊以連續時間密度表示，離散化在濾波器內完成：
 *   Q_θ = σ_g²·dt、Q_b = σ_b²·dt；加速度與零角速度量測的 R 是每筆量測的變異數。
 */
typedef struct {
    float gyro_noise_dps;    /* 角度隨機遊走 σ_g，dps/√Hz */
    float bias_walk_dps;     /* 零偏隨機遊走 σ_b，dps/s/√Hz */
    float accel_noise;       /* 重力方向量測雜訊 σ_a（單位向量，約等於弧度） */
    float accel_adapt;       /* σ_a 隨 ||a|-1 g| 增加的斜率：σ = σ_a + adapt·||a|-1| */
    float accel_reject_g;    /* ||a|-1 g| 超過此值不採用加速度 */
    float zupt_noise_dps;    /* 靜止時角速度量測雜訊（每筆） */
    float init_angle_deg;    /* 初始 roll/pitch 不確定度 */
    float init_bias_dps;     /* 初始零偏不確定度 */
    float still_var_dps;     /* 靜止偵測：原始角速度的標準差上限 */
    float still_dps;         /* 靜止偵測：扣掉估計零偏後的角速度上限 */
    float still_g;           /* 靜止偵測：||a|-1 g| 上限 */
    float still_s;           /* 靜止偵測：需持續的時間 */
    float zupt_enable;       /* 非 0 時靜止才做零角速度量測 */
} ekf_ahrs_params_t;

/* ICM-20948，±500 dps、DLPF 51 Hz，已留餘量給震動。 */
#define EKF_AHRS_DEFAULT_PARAMS { \
    .gyro_noise_dps = 0.05f, .bias_walk_dps = 0.002f, \
    .accel_noise = 0.02f, .accel_adapt = 5.0f, .accel_reject_g = 0.3f, \
    .zupt_noise_dps = 0.1f, .init_angle_deg = 5.0f, .init_bias_dps = 1.0f, \
    .still_var_dps = 0.3f, .still_dps = 3.0f, .still_g = 0.05f, .still_s = 0.5f, \
    .zupt_enable = 1.0f, \
}

/* 最近一步的內部量，供 ahrs_bench 視覺化；不影響濾波。 */
typedef struct {
    float P_prior[6][6];     /* 預測後、修正前的共變異數 */
    float v[3];              /* 修正前預測的重力方向（本體座標） */
    float a_unit[3];         /* 量到的重力方向 */
    float y[3];              /* 加速度創新量 a_unit - v */
    float S[3][3];           /* 創新共變異數 H P Hᵀ + R */
    float K[6][3];           /* 加速度修正的卡爾曼增益 */
    float dx[6];             /* 加速度修正量 [δθ, δb] */
    float sigma_a;           /* 這一步採用的 σ_a */
    float nis;               /* yᵀ S⁻¹ y，模型正確時服從自由度 3 的 χ² */
    float y_zupt[3];         /* 零角速度創新量，rad/s */
    float nis_zupt;
    bool accel_used, zupt_used;
} ekf_ahrs_diag_t;

typedef struct {
    ekf_ahrs_params_t p;
    float q[4];
    float b[3];              /* 零偏，rad/s */
    float P[6][6];
    /* 靜止偵測 */
    float mean[3], var[3];   /* 原始角速度的指數移動平均與變異數，(rad/s)、(rad/s)² */
    uint32_t quiet_n;
    bool still;
    /* 統計 */
    uint32_t accel_updates, accel_rejects, zupt_updates;
    ekf_ahrs_diag_t diag;
} ekf_ahrs_t;

/* 以加速度（重力方向）決定初始 roll/pitch，yaw = 0，零偏 = 0；p 為 NULL 時用預設參數。 */
void ekf_ahrs_init(ekf_ahrs_t *f, const ekf_ahrs_params_t *p, const float accel_g[3]);

/* gyro_dps 為原始讀值（不要先扣零偏）；dt 為取樣間隔，秒。 */
void ekf_ahrs_step(ekf_ahrs_t *f, const float gyro_dps[3], const float accel_g[3], float dt);

/* ZYX 尤拉角，度；yaw 範圍 -180..+180。 */
void ekf_ahrs_euler(const ekf_ahrs_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg);

void ekf_ahrs_bias_dps(const ekf_ahrs_t *f, float bias_dps[3]);

/* 1σ 不確定度：姿態誤差（本體座標，度；姿態接近水平時約等於 roll/pitch/yaw）與零偏（dps）。 */
void ekf_ahrs_sigma(const ekf_ahrs_t *f, float angle_deg[3], float bias_dps[3]);

#endif /* EKF_AHRS_H */
