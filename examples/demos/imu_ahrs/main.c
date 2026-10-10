/*
 * IMU 姿態（AHRS）：以 Mahony 濾波融合陀螺儀與加速度計，估測 roll、pitch、yaw，
 * 並在靜止時持續更新零偏、鎖住角度。是 imu_yaw 的完整版。
 *
 * 處理流程（控制中斷 TIM6，200 Hz）：
 *   讀 IMU → 零偏校正／扣零偏與靜止偵測（gyro_cal）→ Mahony 更新（ahrs）→ 尤拉角
 *   - 開機靜置 2 s 校正三軸零偏，並記下當時的傾角當作水平基準（roll/pitch 相對它顯示）。
 *   - 靜止時：零偏以 10 s 時間常數追蹤目前讀值；姿態不積分陀螺儀，角度不漂。
 *   - 加減速或碰撞使加速度偏離 1 g 時，暫停加速度修正。
 *
 * OLED 三頁，單擊切換：
 *   1 Attitude  人工地平儀與 roll／pitch／yaw
 *   2 Heading   航向儀、角速度、歸零後的漂移
 *   3 Sensors   三軸零偏、加速度大小、溫度、靜止時間、零偏更新次數、中斷時間
 * 長按：yaw 歸零；雙擊：重新校正（要靜置）。
 *
 * 角度依 ROS REP-103（見 ahrs.h），以 IMU 本體座標表示，假設控制板元件面朝上。
 * 依賴（皆已通過功能驗證）：12_key_buzzer、21_oled、22_imu。
 */
#include <math.h>
#include <stdio.h>

#include "ahrs.h"
#include "board.h"
#include "board_id.h"
#include "ctrl_tick.h"
#include "gauge.h"
#include "gyro_cal.h"
#include "imu.h"
#include "key.h"
#include "oled.h"

#define CTRL_HZ          200U
#define OLED_PERIOD_MS   200U
#define LOG_PERIOD_MS    500U
#define PAGE_COUNT       3U

typedef struct {
    gyro_cal_state_t cal_state;
    bool calibrated, still, accel_used;
    float roll, pitch, yaw;        /* roll/pitch 已扣水平基準、yaw 已扣歸零偏移，度 */
    double yaw_total;              /* 歸零後的累積 yaw（不折返），算漂移用 */
    float rate_dps[3];             /* 扣零偏後的角速度 */
    float bias_dps[3];
    float accel_norm, temp_c, still_s, last_std;
    uint32_t bias_updates, read_errors;
} state_t;

/* 由控制中斷寫入，主迴圈關中斷複製一份後使用。 */
static volatile state_t s_state;
static gyro_cal_t s_cal;
static float s_roll0, s_pitch0;        /* 水平基準 */
static float s_yaw_offset, s_last_yaw;
static volatile bool s_req_zero, s_req_cal;

static float wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg <= -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static void ctrl_step(void)
{
    imu_sample_t s;
    float rate[3];

    if (s_req_cal) {
        s_req_cal = false;
        gyro_cal_start(&s_cal, (float)CTRL_HZ);
    }
    if (!imu_read(&s)) {
        s_state.read_errors++;
        return;
    }
    const gyro_cal_state_t st = gyro_cal_feed(&s_cal, &s, rate);
    s_state.cal_state = st;
    s_state.calibrated = s_cal.calibrated;
    s_state.last_std = s_cal.last_std_dps;
    s_state.temp_c = s.temp_c;
    s_state.accel_norm = sqrtf(s.accel_g[0] * s.accel_g[0] + s.accel_g[1] * s.accel_g[1] +
                               s.accel_g[2] * s.accel_g[2]);
    if (!s_cal.calibrated) {
        return;
    }

    float roll, pitch, yaw;
    if (st == GYRO_CAL_DONE) {
        /* 從校正時的平均加速度決定初始姿態與水平基準，yaw 歸零。 */
        ahrs_reset(s_cal.level_g);
        ahrs_euler(&s_roll0, &s_pitch0, &yaw);
        s_yaw_offset = yaw;
        s_last_yaw = yaw;
        s_state.yaw_total = 0.0;
    } else {
        ahrs_update(rate, s.accel_g, 1.0f / (float)CTRL_HZ, s_cal.still);
    }
    ahrs_euler(&roll, &pitch, &yaw);

    if (s_req_zero) {
        s_req_zero = false;
        s_yaw_offset = yaw;
        s_state.yaw_total = 0.0;
    }
    s_state.yaw_total += (double)wrap180(yaw - s_last_yaw);
    s_last_yaw = yaw;

    s_state.roll = roll - s_roll0;
    s_state.pitch = pitch - s_pitch0;
    s_state.yaw = wrap180(yaw - s_yaw_offset);
    for (int k = 0; k < 3; k++) {
        s_state.rate_dps[k] = rate[k];
        s_state.bias_dps[k] = s_cal.bias_dps[k];
    }
    s_state.still = s_cal.still;
    s_state.still_s = s_cal.still_s;
    s_state.bias_updates = s_cal.bias_updates;
    s_state.accel_used = ahrs_accel_used();
}

static state_t snapshot(void)
{
    state_t s;
    __disable_irq();
    s = *(const state_t *)&s_state;
    __enable_irq();
    return s;
}

static void title(const char *name, unsigned page)
{
    char line[24];
    snprintf(line, sizeof line, " %-15s %u/%u", name, page % PAGE_COUNT + 1U, PAGE_COUNT);
    oled_text(0, 0, line);
    oled_invert_row(0);
}

static void page_attitude(const state_t *s)
{
    char line[16];

    title("Attitude", 0);
    snprintf(line, sizeof line, "R %+6.1f", (double)s->roll);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "P %+6.1f", (double)s->pitch);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "Y %+6.1f", (double)s->yaw);
    oled_text(0, 3, line);
    oled_text(0, 5, s->still ? "STILL" : (s->accel_used ? "moving" : "accel off"));
    if (s->still) {
        oled_invert_row(5);
    }
    gauge_horizon(98, 33, 22, s->roll, s->pitch, 1.0f);
}

static void page_heading(const state_t *s, uint32_t zero_ms)
{
    char line[16];
    const float minutes = (float)(HAL_GetTick() - zero_ms) / 60000.0f;

    title("Heading", 1);
    snprintf(line, sizeof line, "Yaw %+7.1f", (double)s->yaw);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "dps %+7.1f", (double)s->rate_dps[2]);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "drft%+6.2f", minutes > 0.1f ? s->yaw_total / (double)minutes : 0.0);
    oled_text(0, 4, line);
    oled_text(0, 5, "  deg/min");
    gauge_heading(104, 35, 20, s->yaw);
}

static void page_sensors(const state_t *s)
{
    char line[24];

    title("Sensors", 2);
    snprintf(line, sizeof line, "bias x%+5.2f y%+5.2f", (double)s->bias_dps[0], (double)s->bias_dps[1]);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "     z%+6.3f dps", (double)s->bias_dps[2]);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "|a| %.3f g %5.1f C", (double)s->accel_norm, (double)s->temp_c);
    oled_text(0, 3, line);
    snprintf(line, sizeof line, "still %6.1f s", (double)s->still_s);
    oled_text(0, 4, line);
    snprintf(line, sizeof line, "updates %lu", (unsigned long)s->bias_updates);
    oled_text(0, 5, line);
    snprintf(line, sizeof line, "isr %lu/%lu us", (unsigned long)ctrl_tick_max_us(),
             (unsigned long)(1000000U / CTRL_HZ));
    oled_text(0, 6, line);
}

static void render(const state_t *s, unsigned page, uint32_t zero_ms)
{
    oled_clear();
    if (!s->calibrated) {
        title("Calibrating", page);
        oled_text(0, 2, "keep still...");
        if (s->last_std > 0.0f) {
            char line[24];
            snprintf(line, sizeof line, "last std %.2f dps", (double)s->last_std);
            oled_text(0, 4, line);
        }
    } else if (page == 0) {
        page_attitude(s);
    } else if (page == 1) {
        page_heading(s, zero_ms);
    } else {
        page_sensors(s);
    }
    oled_text(0, 7, "1x:page hold:0 2x:cal");
    oled_flush();
}

int main(void)
{
    board_init();
    printf("\n==== imu_ahrs ====\n");

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
    printf("Mahony AHRS at %u Hz, Kp %.1f; keep still for %.0f s to calibrate\n",
           CTRL_HZ, (double)AHRS_KP, (double)GYRO_CAL_SECONDS);

    gyro_cal_start(&s_cal, (float)CTRL_HZ);
    ctrl_tick_start(CTRL_HZ, ctrl_step);

    unsigned page = 0;
    bool was_calibrated = false, was_still = false;
    uint32_t zero_ms = HAL_GetTick(), last_oled = 0, last_log = 0;
    for (;;) {
        const uint32_t now = HAL_GetTick();
        const state_t s = snapshot();

        if (s.calibrated != was_calibrated) {
            was_calibrated = s.calibrated;
            if (s.calibrated) {
                zero_ms = now;
                printf("calibrated: bias [%+.3f %+.3f %+.3f] dps, level roll %+.1f pitch %+.1f deg\n",
                       (double)s.bias_dps[0], (double)s.bias_dps[1], (double)s.bias_dps[2],
                       (double)s_roll0, (double)s_pitch0);
            }
        }
        if (s.calibrated && s.still != was_still) {
            was_still = s.still;
            printf("%s\n", s.still ? "still: bias tracking, attitude held" : "moving");
        }
        switch (key_poll()) {
        case KEY_CLICK:
            page = (page + 1) % PAGE_COUNT;
            break;
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
            render(&s, page, zero_ms);
        }
        if (s.calibrated && now - last_log >= LOG_PERIOD_MS) {
            last_log = now;
            printf("rpy %+6.1f %+6.1f %+7.1f  gz %+7.2f  bias_z %+.3f  %s  |a| %.3f  T %.1f  isr %lu us  err %lu\n",
                   (double)s.roll, (double)s.pitch, (double)s.yaw, (double)s.rate_dps[2],
                   (double)s.bias_dps[2], s.still ? "STILL " : "moving", (double)s.accel_norm,
                   (double)s.temp_c, (unsigned long)ctrl_tick_max_us(), (unsigned long)s.read_errors);
        }
        HAL_Delay(10);
    }
}
