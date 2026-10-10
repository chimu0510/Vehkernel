/*
 * ICM-20948 加速度計與陀螺儀（軟體 I2C，腳位見 board.h；暫存器設定與 22_imu 相同）。
 *   加速度 ±2 g、陀螺儀 ±500 dps，兩者 DLPF 51 Hz、輸出率 225 Hz。
 * 一次讀取 14 bytes 約 0.7 ms（軟體 I2C 約 250 kHz）。
 * 匯流排不可重入：同一時間只能有一處（主迴圈或控制中斷）呼叫。
 */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temp_c;
} imu_sample_t;

/* 確認 WHO_AM_I 後重設並設定；失敗回傳 false。 */
bool imu_init(void);

bool imu_read(imu_sample_t *s);

#endif /* IMU_H */
