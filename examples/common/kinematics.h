/*
 * 差速運動學解算與車頭方向映射 (ADR-0001, ADR-0002)
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>
#include "settings.h"  /* heading_direction_t */

/* 預設幾何常數 (待實車標定，見 ADR-0002) */
#define DEFAULT_TRACK_WIDTH_M     0.350f   /* 兩驅動輪有效輪距 (m) */
#define DEFAULT_WHEEL_DIAMETER_M  0.208f   /* 充氣驅動輪直徑 (m) */
#define DEFAULT_COUNTS_PER_REV    36000U   /* 驅動輪轉一圈編碼器計數 (500線 * 4倍頻 * 18減速比) */

typedef struct {
    float track_width;          /* 輪距 (m) */
    float wheel_diameter;       /* 輪徑 (m) */
    uint32_t counts_per_rev;    /* 單輪每圈編碼器總計數 */
    heading_t heading;          /* 車頭方向 (ADR-0001) */
} kinematics_config_t;

void kinematics_init(kinematics_config_t *cfg, float track_width, float wheel_diameter, heading_t heading);
void kinematics_init_full(kinematics_config_t *cfg, float track_width, float wheel_diameter, uint32_t counts_per_rev, heading_t heading);

/**
 * 逆運動學：由車體速度 (Vx, wz) 計算左右馬達的目標線速度 (m/s)
 * 已自動根據車頭方向在映射層處理左右對調與反向 (ADR-0001)。
 */
void kinematics_body_to_motors(const kinematics_config_t *cfg, float vx, float wz,
                               float *out_motor_l_vel, float *out_motor_r_vel);

/**
 * 正運動學：由左右馬達量測的線速度 (m/s) 解算回以 base_link 表示的車體速度 (Vx, wz)
 * 已自動根據車頭方向進行換算。
 */
void kinematics_motors_to_body(const kinematics_config_t *cfg, float motor_l_vel, float motor_r_vel,
                               float *out_vx, float *out_wz);

/**
 * 由編碼器計數差值計算單輪行駛距離 (m)
 */
float kinematics_counts_to_distance(const kinematics_config_t *cfg, int32_t counts);

/**
 * 由單輪行駛距離計算對應的編碼器計數
 */
int32_t kinematics_distance_to_counts(const kinematics_config_t *cfg, float distance_m);

#endif /* KINEMATICS_H */
