/*
 * ahrs_bench 與主機（tools/ahrs_bench）之間的訊息。每個訊框的負載第一個 byte 是訊息類型，
 * 其餘欄位 little endian、無填充（packed）；分幀規則見 frame.h。
 * 主機端的對應定義在 tools/ahrs_bench/protocol.py，兩邊要一起改並遞增 BENCH_PROTO_VERSION。
 *
 * 這是本工具專用的簡化協議，不是 ADR-0003 的控制板與 Raspberry Pi 5 協議。
 */
#ifndef BENCH_PROTO_H
#define BENCH_PROTO_H

#include <stdint.h>

#define BENCH_PROTO_VERSION   1U
#define BENCH_BAUD            921600U
#define BENCH_PARAM_NAME_LEN  14U

/* 控制板 → 主機 */
#define MSG_HELLO   0x01U
#define MSG_SAMPLE  0x10U
#define MSG_PARAMS  0x20U
#define MSG_TEXT    0x30U

/* 主機 → 控制板 */
#define MSG_SET_PARAMS 0x80U
#define MSG_CMD        0x81U

#define CMD_RESET_EKF     1U
#define CMD_RESET_MAHONY  2U
#define CMD_RESET_BOTH    3U
#define CMD_MAHONY_CAL    4U   /* 靜置 2 s 取陀螺儀平均當 Mahony 的零偏 */
#define CMD_HELLO         5U   /* 回 HELLO 與 PARAMS */

/* MSG_SAMPLE.flags */
#define SF_EKF_INIT       0x01U  /* 這一筆用來初始化 EKF（只用加速度，不做 step） */
#define SF_MAH_INIT       0x02U  /* 這一筆用來初始化 Mahony */
#define SF_MAH_BIAS_SET   0x04U  /* 這一筆把 Mahony 零偏設為 mah_b，不做 update */
#define SF_STILL          0x08U  /* EKF 判定靜止 */
#define SF_ACCEL_USED     0x10U  /* EKF 採用了加速度修正 */
#define SF_ZUPT_USED      0x20U  /* EKF 做了零角速度修正 */
#define SF_MAH_ACCEL_USED 0x40U
#define SF_IMU_ERROR      0x80U  /* IMU 讀取失敗：原始值無效，兩個濾波器都沒有執行 */

typedef struct __attribute__((packed)) {
    uint8_t type;            /* MSG_HELLO */
    uint8_t proto_version;
    uint16_t rate_hz;
    uint8_t param_count;
    uint8_t hw_version;      /* board_id 的硬體版本 */
    char name[16];
} msg_hello_t;

typedef struct __attribute__((packed)) {
    uint8_t type;            /* MSG_SAMPLE */
    uint8_t flags;
    uint8_t ekf_epoch;       /* 每次 EKF 初始化加 1 */
    uint8_t mah_epoch;
    uint8_t param_rev;       /* 這一筆使用的參數版本，見 MSG_PARAMS */
    uint32_t seq;            /* 控制中斷計數，時間 = seq / rate_hz */
    float accel_g[3];
    float gyro_dps[3];
    float temp_c;
    float ekf_q[4];
    float ekf_b[3];          /* rad/s */
    float ekf_sig_th[3];     /* sqrt(diag P)，rad */
    float ekf_sig_b[3];      /* rad/s */
    float ekf_nis;
    float mah_q[4];
    float mah_b[3];          /* rad/s */
    uint16_t isr_us;
    uint16_t overruns;
} msg_sample_t;

typedef struct __attribute__((packed)) {
    char name[BENCH_PARAM_NAME_LEN];
    float value;
} bench_param_t;

_Static_assert(sizeof(msg_hello_t) == 22, "msg_hello_t layout changed: update protocol.py");
_Static_assert(sizeof(msg_sample_t) == 125, "msg_sample_t layout changed: update protocol.py");
_Static_assert(sizeof(bench_param_t) == 18, "bench_param_t layout changed: update protocol.py");

/* MSG_PARAMS：uint8 type、uint8 rev、uint8 count，接著 count 個 bench_param_t。 */
/* MSG_SET_PARAMS：uint8 type、uint8 count，接著 count 個 float（順序同 MSG_PARAMS）。 */
/* MSG_CMD：uint8 type、uint8 cmd。 */
/* MSG_TEXT：uint8 type，接著 UTF-8 文字（不含結尾 0）。 */

#endif /* BENCH_PROTO_H */
