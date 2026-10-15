/*
 * 陀螺儀零偏：開機靜置校正，之後偵測靜止並持續更新零偏。
 *
 * 校正：收集 GYRO_CAL_SECONDS 秒，三軸角速度與加速度大小的標準差都夠小才採用平均值，
 *       有晃動就整批重來。同時記下加速度平均，作為水平基準。
 * 靜止偵測：扣掉零偏後三軸角速度都小於 GYRO_STILL_DPS、加速度大小接近 1 g，
 *       而且連續維持 GYRO_STILL_SECONDS 秒。
 * 線上更新：靜止時以時間常數 GYRO_TRACK_TAU_S 的一階濾波把零偏拉向目前讀值，
 *       追蹤溫度與安裝應力造成的零偏變化。
 * 限制：比 GYRO_STILL_DPS 還慢的真實轉動會被當成零偏吃掉；主韌體應再加上
 *       「編碼器顯示兩輪都沒動」的條件。
 */
#ifndef GYRO_CAL_H
#define GYRO_CAL_H

#include <stdbool.h>
#include <stdint.h>

#include "imu.h"

#define GYRO_CAL_SECONDS     2.0f
#define GYRO_CAL_MAX_STD_DPS 0.3f
#define GYRO_CAL_MAX_STD_G   0.02f
#define GYRO_STILL_DPS       1.5f
#define GYRO_STILL_G         0.05f
#define GYRO_STILL_SECONDS   1.0f
#define GYRO_TRACK_TAU_S     10.0f

typedef enum {
    GYRO_CAL_BUSY,       /* 校正中 */
    GYRO_CAL_RETRY,      /* 有晃動，已重新開始 */
    GYRO_CAL_DONE,       /* 剛完成 */
    GYRO_CAL_TRACKING,   /* 已校正，正在輸出修正後的角速度 */
} gyro_cal_state_t;

typedef struct {
    /* 結果 */
    float bias_dps[3];
    float level_g[3];          /* 校正時的加速度平均（水平基準） */
    float last_std_dps;        /* 最近一次校正三軸中最大的標準差 */
    bool calibrated;
    bool still;
    float still_s;             /* 已連續靜止的時間 */
    uint32_t bias_updates;     /* 靜止時更新零偏的次數 */
    /* 內部 */
    float hz;
    uint32_t n, target_n, still_n;
    double sum_g[3], sq_g[3], sum_a[3], sum_an, sq_an;
} gyro_cal_t;

/* 開始（或重新開始）校正；hz 是 gyro_cal_feed() 的呼叫頻率。 */
void gyro_cal_start(gyro_cal_t *c, float hz);

/*
 * 每筆取樣呼叫一次。校正中只累加；完成後把扣掉零偏的角速度寫入 out_dps
 * （校正中寫入 0），並做靜止偵測與零偏更新。
 */
gyro_cal_state_t gyro_cal_feed(gyro_cal_t *c, const imu_sample_t *s, float out_dps[3]);

#endif /* GYRO_CAL_H */
