/*
 * IMU 姿態（EKF）：以乘性誤差狀態 EKF 估測姿態與三軸陀螺儀零偏（見 ekf_ahrs.h），
 * 並在同一份 IMU 資料上同時執行 Mahony（imu_ahrs 的做法）比較兩者。
 *
 * 與 Mahony 版的差別：
 *   - 不需要開機靜置校正：零偏從 0 開始，第一次靜止時由零角速度量測收斂。
 *   - 零偏是濾波狀態，三軸都估（z 軸靠靜止時的零角速度量測）；加速度的權重隨 |a| 偏離 1 g 自動降低。
 *   - 輸出各角度與零偏的 1σ 不確定度；沒有磁力計，σ_yaw 會隨時間成長。
 * Mahony 這一路沿用 gyro_cal：要先靜置 2 s 校正，之後靜止時追蹤零偏並鎖住角度。
 *
 * OLED 三頁，單擊切換：
 *   1 EKF        人工地平儀與 roll／pitch／yaw（附 1σ）
 *   2 EKF vs M   兩者的 yaw、差值、歸零後漂移與 z 軸零偏
 *   3 Filter     零偏估計與 σ、靜止狀態、加速度採用／拒絕次數、零角速度更新次數、中斷時間
 * 長按：兩者的 yaw 一起歸零；雙擊：兩者都重新開始（EKF 由目前加速度重新初始化）。
 *
 * 角度依 ROS REP-103，IMU 本體座標，假設控制板元件面朝上；roll/pitch 為絕對值（不扣水平基準）。
 * 依賴（皆已通過功能驗證）：12_key_buzzer、21_oled、22_imu。
 */
#include <math.h>
#include <stdio.h>

#include "ahrs.h"
#include "board.h"
#include "board_id.h"
#include "ctrl_tick.h"
#include "ekf_ahrs.h"
#include "gauge.h"
#include "gyro_cal.h"
#include "imu.h"
#include "key.h"
#include "oled.h"

#define CTRL_HZ          200U
#define DT               (1.0f / (float)CTRL_HZ)
#define OLED_PERIOD_MS   200U
#define LOG_PERIOD_MS    500U
#define PAGE_COUNT       3U

typedef struct {
    float roll, pitch, yaw;        /* EKF，yaw 已扣歸零偏移 */
    float sig_angle[3], sig_bias[3], bias[3];
    bool still;
    uint32_t accel_updates, accel_rejects, zupt_updates;
    double ekf_total, mah_total;   /* 歸零後的累積 yaw（不折返） */
    float mah_yaw, mah_bias_z;
    bool mah_ready;
    uint32_t read_errors;
} state_t;

static volatile state_t s_state;
static ekf_ahrs_t s_ekf;
static gyro_cal_t s_cal;
static bool s_ekf_ready;
static float s_ekf_off, s_ekf_last, s_mah_off, s_mah_last;
static volatile bool s_req_zero, s_req_reset;

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
    float roll, pitch, yaw, rate[3];

    if (!imu_read(&s)) {
        s_state.read_errors++;
        return;
    }
    if (s_req_reset || !s_ekf_ready) {
        s_req_reset = false;
        ekf_ahrs_init(&s_ekf, s.accel_g);
        gyro_cal_start(&s_cal, (float)CTRL_HZ);
        s_ekf_ready = true;
        s_ekf_off = s_ekf_last = 0.0f;
        s_state.ekf_total = s_state.mah_total = 0.0;
        s_state.mah_ready = false;
        return;
    }

    /* EKF：吃原始角速度。 */
    ekf_ahrs_step(&s_ekf, s.gyro_dps, s.accel_g, DT);
    ekf_ahrs_euler(&s_ekf, &roll, &pitch, &yaw);

    /* Mahony：先由 gyro_cal 扣零偏，校正完成前不執行。 */
    const gyro_cal_state_t cs = gyro_cal_feed(&s_cal, &s, rate);
    float mroll, mpitch, myaw = 0.0f;
    if (cs == GYRO_CAL_DONE) {
        ahrs_reset(s_cal.level_g);
        ahrs_euler(&mroll, &mpitch, &myaw);
        s_mah_off = s_mah_last = myaw;
        s_state.mah_total = 0.0;
    } else if (cs == GYRO_CAL_TRACKING) {
        ahrs_update(rate, s.accel_g, DT, s_cal.still);
        ahrs_euler(&mroll, &mpitch, &myaw);
    }

    if (s_req_zero) {
        s_req_zero = false;
        s_ekf_off = yaw;
        s_mah_off = myaw;
        s_state.ekf_total = s_state.mah_total = 0.0;
    }
    s_state.ekf_total += (double)wrap180(yaw - s_ekf_last);
    s_ekf_last = yaw;
    if (s_cal.calibrated) {
        s_state.mah_total += (double)wrap180(myaw - s_mah_last);
        s_mah_last = myaw;
        s_state.mah_yaw = wrap180(myaw - s_mah_off);
        s_state.mah_bias_z = s_cal.bias_dps[2];
    }
    s_state.mah_ready = s_cal.calibrated;

    s_state.roll = roll;
    s_state.pitch = pitch;
    s_state.yaw = wrap180(yaw - s_ekf_off);
    float sa[3], sb[3], b[3];
    ekf_ahrs_sigma(&s_ekf, sa, sb);
    ekf_ahrs_bias_dps(&s_ekf, b);
    for (int k = 0; k < 3; k++) {
        s_state.sig_angle[k] = sa[k];
        s_state.sig_bias[k] = sb[k];
        s_state.bias[k] = b[k];
    }
    s_state.still = s_ekf.still;
    s_state.accel_updates = s_ekf.accel_updates;
    s_state.accel_rejects = s_ekf.accel_rejects;
    s_state.zupt_updates = s_ekf.zupt_updates;
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

static float drift_per_min(double total, uint32_t zero_ms)
{
    const float minutes = (float)(HAL_GetTick() - zero_ms) / 60000.0f;
    return minutes > 0.1f ? (float)(total / (double)minutes) : 0.0f;
}

static void page_ekf(const state_t *s)
{
    char line[16];

    title("EKF", 0);
    snprintf(line, sizeof line, "R%+6.1f", (double)s->roll);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, " +-%4.2f", (double)s->sig_angle[0]);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "P%+6.1f", (double)s->pitch);
    oled_text(0, 3, line);
    snprintf(line, sizeof line, " +-%4.2f", (double)s->sig_angle[1]);
    oled_text(0, 4, line);
    snprintf(line, sizeof line, "Y%+6.1f", (double)s->yaw);
    oled_text(0, 5, line);
    snprintf(line, sizeof line, " +-%4.1f", (double)s->sig_angle[2]);
    oled_text(0, 6, line);
    if (s->still) {
        oled_text(8, 6, "STILL");
    }
    gauge_horizon(100, 33, 22, s->roll, s->pitch, 1.0f);
}

static void page_compare(const state_t *s, uint32_t zero_ms)
{
    char line[24];

    title("EKF vs Mahony", 1);
    oled_text(0, 1, "       EKF  Mahony");
    if (!s->mah_ready) {
        snprintf(line, sizeof line, "yaw %+6.1f  (cal)", (double)s->yaw);
        oled_text(0, 2, line);
        oled_text(0, 4, "Mahony: keep still");
        return;
    }
    snprintf(line, sizeof line, "yaw %+6.1f %+6.1f", (double)s->yaw, (double)s->mah_yaw);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "drft%+6.2f %+6.2f", (double)drift_per_min(s->ekf_total, zero_ms),
             (double)drift_per_min(s->mah_total, zero_ms));
    oled_text(0, 3, line);
    snprintf(line, sizeof line, "bz  %+6.3f %+6.3f", (double)s->bias[2], (double)s->mah_bias_z);
    oled_text(0, 4, line);
    snprintf(line, sizeof line, "diff %+6.2f deg", (double)wrap180(s->yaw - s->mah_yaw));
    oled_text(0, 5, line);
    oled_text(0, 6, "drft deg/min bz dps");
}

static void page_filter(const state_t *s)
{
    char line[24];

    title("Filter", 2);
    snprintf(line, sizeof line, "b %+5.2f%+5.2f%+6.3f", (double)s->bias[0], (double)s->bias[1], (double)s->bias[2]);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "sb %4.3f %4.3f %4.3f", (double)s->sig_bias[0], (double)s->sig_bias[1],
             (double)s->sig_bias[2]);
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "%s zupt %lu", s->still ? "STILL " : "moving", (unsigned long)s->zupt_updates);
    oled_text(0, 3, line);
    snprintf(line, sizeof line, "acc ok %lu", (unsigned long)s->accel_updates);
    oled_text(0, 4, line);
    snprintf(line, sizeof line, "acc rej %lu", (unsigned long)s->accel_rejects);
    oled_text(0, 5, line);
    snprintf(line, sizeof line, "isr %lu/%lu us", (unsigned long)ctrl_tick_max_us(),
             (unsigned long)(1000000U / CTRL_HZ));
    oled_text(0, 6, line);
}

static void render(const state_t *s, unsigned page, uint32_t zero_ms)
{
    oled_clear();
    if (page == 0) {
        page_ekf(s);
    } else if (page == 1) {
        page_compare(s, zero_ms);
    } else {
        page_filter(s);
    }
    oled_text(0, 7, "1x:page hold:0 2x:rst");
    oled_flush();
}

int main(void)
{
    board_init();
    printf("\n==== imu_ekf ====\n");

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
    printf("MEKF + Mahony at %u Hz; EKF needs no calibration, Mahony needs %.0f s still\n",
           CTRL_HZ, (double)GYRO_CAL_SECONDS);

    ctrl_tick_start(CTRL_HZ, ctrl_step);

    unsigned page = 0;
    bool was_still = false, was_mah = false;
    uint32_t zero_ms = HAL_GetTick(), last_oled = 0, last_log = 0;
    for (;;) {
        const uint32_t now = HAL_GetTick();
        const state_t s = snapshot();

        if (s.still != was_still) {
            was_still = s.still;
            printf("%s\n", s.still ? "still: zero-rate updates on" : "moving");
        }
        if (s.mah_ready != was_mah) {
            was_mah = s.mah_ready;
            if (s.mah_ready) {
                zero_ms = now;
                printf("mahony calibrated; drift counters restarted\n");
            }
        }
        switch (key_poll()) {
        case KEY_CLICK:
            page = (page + 1U) % PAGE_COUNT;
            break;
        case KEY_LONG:
            s_req_zero = true;
            zero_ms = now;
            printf("yaw zeroed\n");
            break;
        case KEY_DOUBLE:
            s_req_reset = true;
            zero_ms = now;
            printf("reset both filters: keep still\n");
            break;
        default:
            break;
        }
        if (has_oled && now - last_oled >= OLED_PERIOD_MS) {
            last_oled = now;
            render(&s, page, zero_ms);
        }
        if (now - last_log >= LOG_PERIOD_MS) {
            last_log = now;
            printf("ekf rpy %+6.1f %+6.1f %+7.2f sig %.2f/%.2f/%.2f bz %+.3f(%.3f) | mahony yaw %+7.2f bz %+.3f | "
                   "%s isr %lu us err %lu\n",
                   (double)s.roll, (double)s.pitch, (double)s.yaw, (double)s.sig_angle[0],
                   (double)s.sig_angle[1], (double)s.sig_angle[2], (double)s.bias[2], (double)s.sig_bias[2],
                   (double)s.mah_yaw, (double)s.mah_bias_z, s.still ? "STILL" : "move ",
                   (unsigned long)ctrl_tick_max_us(), (unsigned long)s.read_errors);
        }
        HAL_Delay(10);
    }
}
