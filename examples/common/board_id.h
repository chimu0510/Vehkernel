/*
 * 判斷 C50C 的硬體版本，邏輯與原廠代碼相同：
 *   - IMU 回應 MPU6050       → 舊版（原廠代碼稱 V1.0）
 *   - IMU 回應 ICM-20948     → 再讀版本腳 PE2/PE3/PC13：0 = V1.1，1 = V1.2
 * V1.2 多了 RS485（UART2 PD5/PD6，接 D50A），OLED 因此改接 PD7/PD4/PE1/PE0。
 * 原廠代碼的 V1.x 與 PCB 絲印的 V4.x 沒有公開的對照，這裡只回報原廠代碼的名稱。
 */
#ifndef BOARD_ID_H
#define BOARD_ID_H

#include <stdint.h>

#include "soft_i2c.h"

typedef enum {
    IMU_NONE = 0,
    IMU_ICM20948,
    IMU_MPU6050,
} imu_type_t;

typedef enum {
    HW_UNKNOWN = 0,
    HW_V1_0,
    HW_V1_1,
    HW_V1_2,
} hw_version_t;

typedef struct {
    imu_type_t imu;
    uint8_t imu_who_am_i;
    uint8_t version_bits;   /* bit0 = PE2、bit1 = PE3、bit2 = PC13 */
    hw_version_t version;
} board_id_t;

/* IMU 所在的 I2C 匯流排（PB9 = SCL、PB8 = SDA）。 */
extern const soft_i2c_t g_imu_bus;

/* 需要先呼叫 board_init()；會初始化 IMU 匯流排與版本腳。 */
board_id_t board_id_detect(void);

const char *board_imu_name(imu_type_t imu);
const char *board_version_name(hw_version_t version);

#endif /* BOARD_ID_H */
