#include "board_id.h"

#include "board.h"

#define ICM20948_REG_WHO_AM_I   0x00U
#define ICM20948_WHO_AM_I       0xEAU
#define MPU6050_REG_WHO_AM_I    0x75U
#define MPU6050_WHO_AM_I        0x68U

const soft_i2c_t g_imu_bus = {
    .scl_port = IMU_SCL_PORT,
    .scl_pin = IMU_SCL_PIN,
    .sda_port = IMU_SDA_PORT,
    .sda_pin = IMU_SDA_PIN,
};

static uint8_t read_version_bits(void)
{
    GPIO_InitTypeDef io = {0};

    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLDOWN;
    io.Pin = VER_BIT0_PIN;
    HAL_GPIO_Init(VER_BIT0_PORT, &io);
    io.Pin = VER_BIT1_PIN;
    HAL_GPIO_Init(VER_BIT1_PORT, &io);
    io.Pin = VER_BIT2_PIN;
    HAL_GPIO_Init(VER_BIT2_PORT, &io);
    HAL_Delay(1);

    uint8_t bits = 0;
    bits |= (HAL_GPIO_ReadPin(VER_BIT0_PORT, VER_BIT0_PIN) == GPIO_PIN_SET) ? 1U : 0U;
    bits |= (HAL_GPIO_ReadPin(VER_BIT1_PORT, VER_BIT1_PIN) == GPIO_PIN_SET) ? 2U : 0U;
    bits |= (HAL_GPIO_ReadPin(VER_BIT2_PORT, VER_BIT2_PIN) == GPIO_PIN_SET) ? 4U : 0U;
    return bits;
}

board_id_t board_id_detect(void)
{
    board_id_t id = {0};
    uint8_t who = 0;

    soft_i2c_init(&g_imu_bus);
    id.version_bits = read_version_bits();

    if (soft_i2c_read_regs(&g_imu_bus, ICM20948_I2C_ADDR, ICM20948_REG_WHO_AM_I, &who, 1) &&
        who == ICM20948_WHO_AM_I) {
        id.imu = IMU_ICM20948;
        id.imu_who_am_i = who;
        id.version = (id.version_bits == 0U) ? HW_V1_1
                   : (id.version_bits == 1U) ? HW_V1_2
                   : HW_UNKNOWN;
    } else if (soft_i2c_read_regs(&g_imu_bus, MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, &who, 1) &&
               who == MPU6050_WHO_AM_I) {
        id.imu = IMU_MPU6050;
        id.imu_who_am_i = who;
        id.version = HW_V1_0;
    } else {
        id.imu_who_am_i = who;
    }
    return id;
}

const char *board_imu_name(imu_type_t imu)
{
    switch (imu) {
    case IMU_ICM20948: return "ICM-20948";
    case IMU_MPU6050:  return "MPU6050";
    default:           return "none";
    }
}

const char *board_version_name(hw_version_t version)
{
    switch (version) {
    case HW_V1_0: return "V1.0";
    case HW_V1_1: return "V1.1";
    case HW_V1_2: return "V1.2";
    default:      return "unknown";
    }
}
