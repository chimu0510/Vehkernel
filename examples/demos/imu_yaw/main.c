/*
 * IMU 航向角：以陀螺儀 z 軸積分出航向角（yaw），並量測零偏與漂移。
 *   1. 開機先靜置校正零偏：2 s 內 z 軸角速度的標準差夠小才採用，有晃動就重來。
 *   2. 控制中斷（TIM6，200 Hz）讀 IMU、扣掉零偏後積分；主迴圈只負責顯示與主控台。
 *   3. OLED 與主控台顯示航向角、角速度、零偏、歸零後的平均漂移與中斷執行時間。
 *
 * 航向角以「由上往下看逆時針為正」（IMU_YAW_SIGN，見 board.h），範圍 -180..+180 度。
 * 按鍵：長按 = 航向角歸零；雙擊 = 重新校正零偏（要靜置）。
 *
 * 驗證方式：逆時針轉 90 度應顯示約 +90；轉整整一圈回到記號應回到約 0（誤差反映陀螺儀
 * 刻度）；靜置幾分鐘看漂移（deg/min）。
 * 依賴（皆已通過功能驗證）：12_key_buzzer、21_oled、22_imu。
 */
#include <math.h>
#include <stdio.h>

#include "board.h"
#include "board_id.h"
#include "ctrl_tick.h"
#include "gauge.h"
#include "imu.h"
#include "key.h"
#include "oled.h"

#define CTRL_HZ          200U
#define CAL_SAMPLES      400U     /* 2 s */
#define CAL_MAX_STD_DPS  0.3f     /* 靜置時 z 軸雜訊約 0.1 dps；超過就當成有晃動 */
#define OLED_PERIOD_MS   200U
#define LOG_PERIOD_MS    500U

typedef enum {
    MODE_CALIBRATING,
    MODE_RUNNING,
} yaw_mode_t;

/* 以下由控制中斷寫入，主迴圈讀取前先關中斷複製一份。 */
static volatile yaw_mode_t s_mode;
static volatile float s_bias_dps;
static volatile float s_rate_dps;      /* 扣掉零偏、已套用 IMU_YAW_SIGN */
static volatile double s_yaw_deg;      /* 不折返的累積角度，用 double 避免長時間積分損失精度 */
static volatile uint32_t s_cal_n;
static volatile float s_cal_std;
static volatile uint32_t s_read_errors;
static volatile double s_rate_sum;     /* 角速度累加，主迴圈以差值算區間平均 */
static volatile uint32_t s_rate_n;
static volatile float s_temp_c;
static double s_cal_sum, s_cal_sq;

/* 主迴圈要求的動作，由控制中斷執行，避免兩邊同時改狀態。 */
static volatile bool s_req_zero, s_req_cal;

static void start_calibration(void)
{
    s_cal_n = 0;
    s_cal_sum = s_cal_sq = 0.0;
    s_mode = MODE_CALIBRATING;
}

static void ctrl_step(void)
{
    imu_sample_t s;

    if (s_req_cal) {
        s_req_cal = false;
        start_calibration();
    }
    if (s_req_zero) {
        s_req_zero = false;
        s_yaw_deg = 0.0;
    }
    if (!imu_read(&s)) {
        s_read_errors++;
        return;
    }
    const float gz = s.gyro_dps[2];
    s_temp_c = s.temp_c;

    if (s_mode == MODE_CALIBRATING) {
        s_cal_sum += (double)gz;
        s_cal_sq += (double)gz * (double)gz;
        if (++s_cal_n < CAL_SAMPLES) {
            return;
        }
        const double mean = s_cal_sum / CAL_SAMPLES;
        const float std = (float)sqrt(fmax(s_cal_sq / CAL_SAMPLES - mean * mean, 0.0));
        s_cal_std = std;
        if (std > CAL_MAX_STD_DPS) {
            start_calibration();       /* 有晃動，重來 */
            return;
        }
        s_bias_dps = (float)mean;
        s_yaw_deg = 0.0;
        s_mode = MODE_RUNNING;
        return;
    }

    const float rate = (float)IMU_YAW_SIGN * (gz - s_bias_dps);
    s_rate_dps = rate;
    s_yaw_deg += (double)rate / CTRL_HZ;
    s_rate_sum += (double)rate;
    s_rate_n++;
}

typedef struct {
    yaw_mode_t mode;
    float bias, rate, cal_std, temp;
    double yaw, rate_sum;
    uint32_t cal_n, errors, rate_n;
} snapshot_t;

static snapshot_t snapshot(void)
{
    snapshot_t s;
    __disable_irq();
    s.mode = s_mode;
    s.bias = s_bias_dps;
    s.rate = s_rate_dps;
    s.cal_std = s_cal_std;
    s.yaw = s_yaw_deg;
    s.cal_n = s_cal_n;
    s.errors = s_read_errors;
    s.rate_sum = s_rate_sum;
    s.rate_n = s_rate_n;
    s.temp = s_temp_c;
    __enable_irq();
    return s;
}

/* 折返到 -180..+180。 */
static float wrap180(double deg)
{
    double w = fmod(deg, 360.0);
    if (w > 180.0) {
        w -= 360.0;
    } else if (w <= -180.0) {
        w += 360.0;
    }
    return (float)w;
}

static void render(const snapshot_t *s, uint32_t zero_ms)
{
    char line[24];

    oled_clear();
    oled_text(0, 0, " IMU yaw");
    oled_invert_row(0);
    if (s->mode == MODE_CALIBRATING) {
        oled_text(0, 2, "keep still...");
        snprintf(line, sizeof line, "calibrating %3lu%%", (unsigned long)(s->cal_n * 100U / CAL_SAMPLES));
        oled_text(0, 3, line);
        if (s->cal_std > 0.0f) {
            snprintf(line, sizeof line, "last std %.2f dps", (double)s->cal_std);
            oled_text(0, 4, line);
        }
        oled_flush();
        return;
    }
    const float yaw = wrap180(s->yaw);
    const float minutes = (float)(HAL_GetTick() - zero_ms) / 60000.0f;
    snprintf(line, sizeof line, "Yaw %+7.1f", (double)yaw);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "dps %+7.1f", (double)s->rate);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "bias%+6.2f", (double)s->bias);
    oled_text(0, 3, line);
    snprintf(line, sizeof line, "drft%+6.2f", minutes > 0.1f ? (double)((float)s->yaw / minutes) : 0.0);
    oled_text(0, 4, line);
    oled_text(0, 5, "  deg/min");
    snprintf(line, sizeof line, "isr %3lu/%lu us", (unsigned long)ctrl_tick_max_us(),
             (unsigned long)(1000000U / CTRL_HZ));
    oled_text(0, 6, line);
    oled_text(0, 7, "hold:zero 2x:recal");
    gauge_heading(104, 35, 20, yaw);
    oled_flush();
}

int main(void)
{
    board_init();
    printf("\n==== imu_yaw ====\n");

    const board_id_t id = board_id_detect();
    const bool has_oled = id.version == HW_V1_0 || id.version == HW_V1_1;
    if (has_oled) {
        oled_init();
    }
    if (!imu_init()) {
        printf("IMU init failed (WHO_AM_I or configuration); run 22_imu\n");
        for (;;) {
        }
    }
    printf("yaw: counter-clockwise seen from above is positive (IMU_YAW_SIGN %+d)\n", IMU_YAW_SIGN);
    printf("keep the car still for 2 s to calibrate the gyro bias\n");

    start_calibration();
    ctrl_tick_start(CTRL_HZ, ctrl_step);

    yaw_mode_t last_mode = MODE_CALIBRATING;
    uint32_t zero_ms = HAL_GetTick(), last_oled = 0, last_log = 0;
    double log_sum = 0.0;
    uint32_t log_n = 0;
    for (;;) {
        const uint32_t now = HAL_GetTick();
        const snapshot_t s = snapshot();

        if (s.mode != last_mode) {
            last_mode = s.mode;
            if (s.mode == MODE_RUNNING) {
                zero_ms = now;
                printf("bias %+.3f dps (std %.3f dps over %u samples)\n",
                       (double)s.bias, (double)s.cal_std, CAL_SAMPLES);
            }
        }
        switch (key_poll()) {
        case KEY_LONG:
            s_req_zero = true;
            zero_ms = now;
            printf("yaw zeroed\n");
            break;
        case KEY_DOUBLE:
            s_req_cal = true;
            printf("recalibrating: keep still\n");
            break;
        default:
            break;
        }
        if (has_oled && now - last_oled >= OLED_PERIOD_MS) {
            last_oled = now;
            render(&s, zero_ms);
        }
        if (s.mode == MODE_RUNNING && now - last_log >= LOG_PERIOD_MS) {
            last_log = now;
            /* 區間平均：持續的偏移代表零偏改變（應力或溫度），短暫的尖峰代表真的有轉動。 */
            const uint32_t n = s.rate_n - log_n;
            const double mean = n > 0U ? (s.rate_sum - log_sum) / n : 0.0;
            log_sum = s.rate_sum;
            log_n = s.rate_n;
            printf("yaw %+8.2f deg (total %+9.2f)  rate %+7.2f dps  mean %+6.3f dps  temp %.2f C"
                   "  isr %lu/%lu us  overruns %lu  i2c errors %lu\n",
                   (double)wrap180(s.yaw), s.yaw, (double)s.rate, mean, (double)s.temp,
                   (unsigned long)ctrl_tick_last_us(), (unsigned long)ctrl_tick_max_us(),
                   (unsigned long)ctrl_tick_overruns(), (unsigned long)s.errors);
        }
        HAL_Delay(10);
    }
}
