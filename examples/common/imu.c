#include "imu.h"

#include <stddef.h>
#include <stdint.h>

#include "board.h"
#include "board_id.h"
#include "soft_i2c.h"

/* Bank 0 */
#define REG_WHO_AM_I        0x00U
#define REG_USER_CTRL       0x03U
#define REG_PWR_MGMT_1      0x06U
#define REG_PWR_MGMT_2      0x07U
#define REG_INT_PIN_CFG     0x0FU
#define REG_ACCEL_XOUT_H    0x2DU   /* accel(6) gyro(6) temp(2)，共 14 bytes，big endian */
#define REG_BANK_SEL        0x7FU
/* Bank 2 */
#define REG_GYRO_SMPLRT_DIV 0x00U
#define REG_GYRO_CONFIG_1   0x01U
#define REG_ACCEL_SMPLRT_DIV_2 0x11U
#define REG_ACCEL_CONFIG    0x14U

#define WHO_AM_I_VALUE      0xEAU
#define PWR_RESET           0x80U
#define PWR_CLK_AUTO        0x01U
#define INT_BYPASS_EN       0x02U
#define DLPF_51HZ           (3U << 3)
#define FCHOICE_DLPF        0x01U
#define GYRO_FS_500DPS      (1U << 1)
#define ACCEL_FS_2G         (0U << 1)
#define SMPLRT_DIV_225HZ    4U      /* 1125 / (1 + 4) */

#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    65.5f
#define TEMP_LSB_PER_C      333.87f
#define TEMP_OFFSET_C       21.0f

static const soft_i2c_t *const bus = &g_imu_bus;

static bool write_reg(uint8_t reg, uint8_t value)
{
    return soft_i2c_write_reg(bus, ICM20948_I2C_ADDR, reg, value);
}

static bool read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    return soft_i2c_read_regs(bus, ICM20948_I2C_ADDR, reg, buf, len);
}

static bool bank(uint8_t b)
{
    return write_reg(REG_BANK_SEL, (uint8_t)(b << 4));
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

bool imu_init(void)
{
    uint8_t who = 0, readback = 0;

    soft_i2c_init(bus);
    if (!bank(0) || !read_regs(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_VALUE ||
        !write_reg(REG_PWR_MGMT_1, PWR_RESET)) {
        return false;
    }
    HAL_Delay(100);
    const uint8_t gyro_cfg = DLPF_51HZ | GYRO_FS_500DPS | FCHOICE_DLPF;
    const bool ok = bank(0) &&
                    write_reg(REG_PWR_MGMT_1, PWR_CLK_AUTO) &&
                    write_reg(REG_PWR_MGMT_2, 0x00) &&          /* 加速度、陀螺儀全開 */
                    write_reg(REG_USER_CTRL, 0x00) &&           /* 關閉內部 I2C 主機 */
                    write_reg(REG_INT_PIN_CFG, INT_BYPASS_EN) && /* 磁力計直接掛在外部匯流排 */
                    bank(2) &&
                    write_reg(REG_GYRO_SMPLRT_DIV, SMPLRT_DIV_225HZ) &&
                    write_reg(REG_GYRO_CONFIG_1, gyro_cfg) &&
                    write_reg(REG_ACCEL_SMPLRT_DIV_2, SMPLRT_DIV_225HZ) &&
                    write_reg(REG_ACCEL_CONFIG, DLPF_51HZ | ACCEL_FS_2G | FCHOICE_DLPF) &&
                    read_regs(REG_GYRO_CONFIG_1, &readback, 1) &&
                    bank(0);
    HAL_Delay(50);
    return ok && readback == gyro_cfg;
}

bool imu_read(imu_sample_t *s)
{
    uint8_t raw[14];

    if (!read_regs(REG_ACCEL_XOUT_H, raw, sizeof raw)) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        s->accel_g[i] = (float)be16(&raw[2 * i]) / ACCEL_LSB_PER_G;
        s->gyro_dps[i] = (float)be16(&raw[6 + 2 * i]) / GYRO_LSB_PER_DPS;
    }
    s->temp_c = (float)be16(&raw[12]) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
    return true;
}
