/*
 * 22 ICM-20948 IMU（軟體 I2C：PB9 = SCL、PB8 = SDA，位址 0x68）。
 *   1. 辨識晶片並設定量程：加速度 ±2 g、角速度 ±500 dps、DLPF 約 51 Hz。
 *   2. 靜止 2 秒：檢查重力大小、陀螺儀零偏與雜訊、溫度。
 *   3. 以 bypass 模式讀內建磁力計 AK09916。
 *   4. 請使用者水平轉動車子，確認 z 軸角速度有反應並記錄正負號。
 *   5. 之後以 5 Hz 持續輸出量測值。
 * 執行前把板子（或整台車）靜置在水平面上。
 */
#include <math.h>
#include <stdio.h>

#include "board.h"
#include "board_id.h"
#include "report.h"
#include "soft_i2c.h"

/* ICM-20948 user bank 0 */
#define REG_WHO_AM_I        0x00U
#define REG_USER_CTRL       0x03U
#define REG_PWR_MGMT_1      0x06U
#define REG_PWR_MGMT_2      0x07U
#define REG_INT_PIN_CFG     0x0FU
#define REG_INT_STATUS_1    0x1AU
#define REG_ACCEL_XOUT_H    0x2DU   /* accel(6) gyro(6) temp(2)，共 14 bytes */
#define REG_BANK_SEL        0x7FU
/* user bank 2 */
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

#define ACCEL_LSB_PER_G     16384.0f
#define GYRO_LSB_PER_DPS    65.5f
#define TEMP_LSB_PER_C      333.87f
#define TEMP_OFFSET_C       21.0f

/* AK09916 磁力計 */
#define MAG_ADDR            0x0CU
#define MAG_WIA1            0x00U
#define MAG_WIA2            0x01U
#define MAG_ST1             0x10U
#define MAG_HXL             0x11U   /* HX HY HZ（little endian）+ dummy + ST2，共 8 bytes */
#define MAG_CNTL2           0x31U
#define MAG_CNTL3           0x32U
#define MAG_MODE_100HZ      0x08U
#define MAG_UT_PER_LSB      0.15f

#define STILL_SAMPLES       200U
#define MOTION_TIMEOUT_MS   15000U
#define MOTION_THRESHOLD_DPS 30.0f

typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temp_c;
} imu_sample_t;

static const soft_i2c_t *const bus = &g_imu_bus;

static bool imu_write(uint8_t reg, uint8_t value)
{
    return soft_i2c_write_reg(bus, ICM20948_I2C_ADDR, reg, value);
}

static bool imu_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return soft_i2c_read_regs(bus, ICM20948_I2C_ADDR, reg, buf, len);
}

static bool imu_bank(uint8_t bank)
{
    return imu_write(REG_BANK_SEL, (uint8_t)(bank << 4));
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)((p[1] << 8) | p[0]);
}

static bool imu_configure(void)
{
    uint8_t readback = 0;

    if (!imu_bank(0) || !imu_write(REG_PWR_MGMT_1, PWR_RESET)) {
        return false;
    }
    HAL_Delay(100);
    const bool ok = imu_bank(0) &&
                    imu_write(REG_PWR_MGMT_1, PWR_CLK_AUTO) &&
                    imu_write(REG_PWR_MGMT_2, 0x00) &&          /* 加速度、陀螺儀全開 */
                    imu_write(REG_USER_CTRL, 0x00) &&           /* 關閉內部 I2C 主機 */
                    imu_write(REG_INT_PIN_CFG, INT_BYPASS_EN) && /* 磁力計直接掛在外部匯流排 */
                    imu_bank(2) &&
                    imu_write(REG_GYRO_SMPLRT_DIV, 10) &&       /* 1125/(1+10) ≈ 102 Hz */
                    imu_write(REG_GYRO_CONFIG_1, DLPF_51HZ | GYRO_FS_500DPS | FCHOICE_DLPF) &&
                    imu_write(REG_ACCEL_SMPLRT_DIV_2, 10) &&
                    imu_write(REG_ACCEL_CONFIG, DLPF_51HZ | ACCEL_FS_2G | FCHOICE_DLPF) &&
                    imu_read(REG_GYRO_CONFIG_1, &readback, 1) &&
                    imu_bank(0);
    HAL_Delay(50);
    return ok && readback == (DLPF_51HZ | GYRO_FS_500DPS | FCHOICE_DLPF);
}

static bool imu_sample(imu_sample_t *s)
{
    uint8_t raw[14];

    if (!imu_read(REG_ACCEL_XOUT_H, raw, sizeof raw)) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        s->accel_g[i] = (float)be16(&raw[2 * i]) / ACCEL_LSB_PER_G;
        s->gyro_dps[i] = (float)be16(&raw[6 + 2 * i]) / GYRO_LSB_PER_DPS;
    }
    s->temp_c = (float)be16(&raw[12]) / TEMP_LSB_PER_C + TEMP_OFFSET_C;
    return true;
}

static bool wait_data_ready(uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    uint8_t status = 0;

    while (HAL_GetTick() - start < timeout_ms) {
        if (imu_read(REG_INT_STATUS_1, &status, 1) && (status & 0x01U)) {
            return true;
        }
    }
    return false;
}

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void test_still(void)
{
    float sum_a[3] = {0}, sum_g[3] = {0}, sq_g[3] = {0}, sum_norm = 0.0f, sq_norm = 0.0f;
    float temp = 0.0f;
    unsigned n = 0;

    printf("  keep the board still for 2 s...\n");
    for (unsigned i = 0; i < STILL_SAMPLES; i++) {
        imu_sample_t s;
        if (!imu_sample(&s)) {
            continue;
        }
        const float a = norm3(s.accel_g);
        for (int k = 0; k < 3; k++) {
            sum_a[k] += s.accel_g[k];
            sum_g[k] += s.gyro_dps[k];
            sq_g[k] += s.gyro_dps[k] * s.gyro_dps[k];
        }
        sum_norm += a;
        sq_norm += a * a;
        temp = s.temp_c;
        n++;
        HAL_Delay(10);
    }
    if (!report_check(n >= STILL_SAMPLES * 9U / 10U, "read samples", "%u/%u", n, STILL_SAMPLES)) {
        return;
    }

    const float g = sum_norm / (float)n;
    const float g_std = sqrtf(fmaxf(sq_norm / (float)n - g * g, 0.0f));
    float bias[3], noise[3];
    for (int k = 0; k < 3; k++) {
        bias[k] = sum_g[k] / (float)n;
        noise[k] = sqrtf(fmaxf(sq_g[k] / (float)n - bias[k] * bias[k], 0.0f));
    }

    printf("  accel mean [%.3f %.3f %.3f] g\n",
           (double)(sum_a[0] / (float)n), (double)(sum_a[1] / (float)n), (double)(sum_a[2] / (float)n));
    report_check(g > 0.90f && g < 1.10f, "gravity", "|a| = %.3f g (expect 0.90..1.10)", (double)g);
    report_check(g_std < 0.02f, "accel noise", "std %.4f g (expect < 0.02)", (double)g_std);
    report_check(fabsf(bias[0]) < 5.0f && fabsf(bias[1]) < 5.0f && fabsf(bias[2]) < 5.0f,
                 "gyro bias", "[%.2f %.2f %.2f] dps (expect |each| < 5)",
                 (double)bias[0], (double)bias[1], (double)bias[2]);
    report_check(noise[0] < 0.5f && noise[1] < 0.5f && noise[2] < 0.5f,
                 "gyro noise", "std [%.3f %.3f %.3f] dps (expect < 0.5)",
                 (double)noise[0], (double)noise[1], (double)noise[2]);
    report_check(temp > 0.0f && temp < 70.0f, "temperature", "%.1f C", (double)temp);
}

static void test_read_time(void)
{
    imu_sample_t s;
    const uint32_t t0 = micros();
    const bool ok = imu_sample(&s);
    const uint32_t dt = micros() - t0;
    report_check(ok && dt < 1500U, "read time", "14-byte burst read %lu us (expect < 1500)",
                 (unsigned long)dt);
}

static void test_magnetometer(void)
{
    uint8_t wia[2] = {0};

    if (!report_check(soft_i2c_read_regs(bus, MAG_ADDR, MAG_WIA1, wia, 2) &&
                      wia[0] == 0x48U && wia[1] == 0x09U,
                      "mag id", "AK09916 WIA 0x%02X 0x%02X (expect 0x48 0x09)", wia[0], wia[1])) {
        return;
    }
    soft_i2c_write_reg(bus, MAG_ADDR, MAG_CNTL3, 0x01); /* soft reset */
    HAL_Delay(10);
    soft_i2c_write_reg(bus, MAG_ADDR, MAG_CNTL2, MAG_MODE_100HZ);

    const uint32_t start = HAL_GetTick();
    uint8_t st1 = 0;
    while (HAL_GetTick() - start < 100U &&
           !(soft_i2c_read_regs(bus, MAG_ADDR, MAG_ST1, &st1, 1) && (st1 & 0x01U))) {
    }
    uint8_t raw[8];
    if (!report_check((st1 & 0x01U) && soft_i2c_read_regs(bus, MAG_ADDR, MAG_HXL, raw, sizeof raw),
                      "mag data ready", "ST1 0x%02X", st1)) {
        return;
    }
    const float m[3] = {
        (float)le16(&raw[0]) * MAG_UT_PER_LSB,
        (float)le16(&raw[2]) * MAG_UT_PER_LSB,
        (float)le16(&raw[4]) * MAG_UT_PER_LSB,
    };
    const float b = norm3(m);
    report_check(b > 10.0f && b < 150.0f, "mag field",
                 "[%.1f %.1f %.1f] uT, |B| = %.1f uT (earth ~25..65; motors and steel distort it)",
                 (double)m[0], (double)m[1], (double)m[2], (double)b);
}

static void test_yaw_motion(void)
{
    const uint32_t start = HAL_GetTick();
    float peak = 0.0f;

    printf("  turn the car (or board) counter-clockwise, seen from above, within %lus\n",
           (unsigned long)(MOTION_TIMEOUT_MS / 1000U));
    while (HAL_GetTick() - start < MOTION_TIMEOUT_MS && fabsf(peak) < MOTION_THRESHOLD_DPS) {
        imu_sample_t s;
        if (imu_sample(&s) && fabsf(s.gyro_dps[2]) > fabsf(peak)) {
            peak = s.gyro_dps[2];
        }
        HAL_Delay(5);
    }
    if (fabsf(peak) < MOTION_THRESHOLD_DPS) {
        report_check(false, "yaw motion", "peak gz %.1f dps (expect |gz| > %.0f)",
                     (double)peak, (double)MOTION_THRESHOLD_DPS);
        return;
    }
    report_check(true, "yaw motion", "peak gz %+.1f dps", (double)peak);
    printf("  note: counter-clockwise gave %s gz -> the board z axis points %s\n",
           peak > 0.0f ? "positive" : "negative", peak > 0.0f ? "up" : "down");
}

static void stream(void)
{
    printf("  streaming at 5 Hz: accel[g] gyro[dps] temp[C]\n");
    for (;;) {
        imu_sample_t s;
        if (imu_sample(&s)) {
            printf("  a %+6.3f %+6.3f %+6.3f  g %+8.2f %+8.2f %+8.2f  t %.1f\n",
                   (double)s.accel_g[0], (double)s.accel_g[1], (double)s.accel_g[2],
                   (double)s.gyro_dps[0], (double)s.gyro_dps[1], (double)s.gyro_dps[2],
                   (double)s.temp_c);
        } else {
            printf("  read failed\n");
        }
        HAL_Delay(200);
    }
}

int main(void)
{
    uint8_t who = 0;

    board_init();
    report_begin("22_imu");
    soft_i2c_init(bus);

    if (!report_check(imu_bank(0) && imu_read(REG_WHO_AM_I, &who, 1) && who == WHO_AM_I_VALUE,
                      "who_am_i", "0x%02X (expect 0x%02X)", who, WHO_AM_I_VALUE) ||
        !report_check(imu_configure(), "configure", "+-2 g, +-500 dps, DLPF 51 Hz, bypass on") ||
        !report_check(wait_data_ready(100), "data ready", "INT_STATUS_1.RAW_DATA_0_RDY")) {
        report_end();
        report_halt();
    }

    test_read_time();
    test_still();
    test_magnetometer();
    test_yaw_motion();
    report_end();

    stream();
}
