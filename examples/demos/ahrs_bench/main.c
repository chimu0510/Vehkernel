/*
 * AHRS Bench（姿態估測工作台）的韌體端：在控制板上執行 EKF（ekf_ahrs）與 Mahony（ahrs），經 USART1 以二進位協議
 * 把每一筆原始量測與兩個濾波器的輸出送到主機的 tools/ahrs_bench（Web UI），並接受主機調整參數、
 * 重設濾波器。主機用同一份 C 代碼重跑一個副本，所以能看到控制板上看不到的內部量
 * （共變異數、創新量、卡爾曼增益），並在相同資料上比較不同參數。
 *
 * 控制中斷 TIM6 200 Hz：讀 IMU → EKF step → Mahony update → 把結果放進佇列。
 * 主迴圈：送出佇列中的樣本、處理主機命令、更新 OLED。
 *
 * 啟動後 USART1 改為 BENCH_BAUD 的二進位協議，一般串口終端機看到的會是亂碼；
 * 開機時先以 115200 印一行提示。
 *
 * EKF 與 Mahony 這裡都直接吃原始角速度、不做開機靜置校正：
 *   - EKF 的零偏是濾波狀態，第一次靜止時由零角速度量測收斂。
 *   - Mahony 只有 Ki > 0 時以積分項估 x、y 零偏；z 軸零偏可用主機的「Mahony 靜置校正」命令設定。
 * 長按使用者按鍵：兩個濾波器都重設。
 * 依賴（皆已通過功能驗證）：12_key_buzzer、21_oled、22_imu。
 */
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "ahrs.h"
#include "board.h"
#include "board_id.h"
#include "ctrl_tick.h"
#include "ekf_ahrs.h"
#include "imu.h"
#include "key.h"
#include "bench_proto.h"
#include "link.h"
#include "oled.h"

#define CTRL_HZ          200U
#define DT               (1.0f / (float)CTRL_HZ)
#define QUEUE_LEN        64U
#define OLED_PERIOD_MS   250U
#define CAL_SAMPLES      (2U * CTRL_HZ)
#define DEG2RAD          (3.14159265f / 180.0f)
#define RAD2DEG          (180.0f / 3.14159265f)

typedef struct {
    ekf_ahrs_params_t ekf;
    ahrs_params_t mah;
} bench_params_t;

/* 參數表：名稱是主機端辨識參數的依據（tools/ahrs_bench/params.py），順序即 MSG_SET_PARAMS 的順序。 */
#define PARAM(group, field, name) {name, offsetof(bench_params_t, group.field)}
static const struct {
    const char *name;
    size_t offset;
} k_params[] = {
    PARAM(ekf, gyro_noise_dps, "ekf.gyro_noise"),
    PARAM(ekf, bias_walk_dps, "ekf.bias_walk"),
    PARAM(ekf, accel_noise, "ekf.acc_noise"),
    PARAM(ekf, accel_adapt, "ekf.acc_adapt"),
    PARAM(ekf, accel_reject_g, "ekf.acc_reject"),
    PARAM(ekf, zupt_noise_dps, "ekf.zupt_noise"),
    PARAM(ekf, init_angle_deg, "ekf.init_angle"),
    PARAM(ekf, init_bias_dps, "ekf.init_bias"),
    PARAM(ekf, still_var_dps, "ekf.still_std"),
    PARAM(ekf, still_dps, "ekf.still_rate"),
    PARAM(ekf, still_g, "ekf.still_acc"),
    PARAM(ekf, still_s, "ekf.still_time"),
    PARAM(ekf, zupt_enable, "ekf.zupt_on"),
    PARAM(mah, kp, "mah.kp"),
    PARAM(mah, ki, "mah.ki"),
    PARAM(mah, accel_gate_g, "mah.acc_gate"),
};
#define PARAM_COUNT (sizeof k_params / sizeof k_params[0])

/* 控制中斷使用的參數；主迴圈改寫 s_pending 後設旗標，由下一次中斷套用。 */
static bench_params_t s_params = {EKF_AHRS_DEFAULT_PARAMS, AHRS_DEFAULT_PARAMS};
static bench_params_t s_pending;
static volatile uint8_t s_pending_rev;
static volatile bool s_pending_set;
static uint8_t s_param_rev;

static ekf_ahrs_t s_ekf;
static ahrs_t s_mah;
static bool s_ekf_ready, s_mah_ready;
static uint8_t s_ekf_epoch, s_mah_epoch;
static uint32_t s_seq;
static volatile bool s_req_reset_ekf, s_req_reset_mah, s_req_cal;

/* Mahony 靜置校正 */
static uint32_t s_cal_n;
static float s_cal_sum[3], s_cal_sq[3];
static volatile bool s_cal_done;
static volatile float s_cal_mean[3], s_cal_std_dps;

/* 中斷 → 主迴圈的樣本佇列（單一生產者、單一消費者） */
static msg_sample_t s_queue[QUEUE_LEN];
static volatile uint32_t s_q_head, s_q_tail, s_q_dropped;

static float *param_ptr(bench_params_t *p, size_t i)
{
    return (float *)((uint8_t *)p + k_params[i].offset);
}

static void mahony_cal_feed(const float gyro_dps[3], bool *bias_set)
{
    if (s_req_cal) {
        s_req_cal = false;
        s_cal_n = CAL_SAMPLES + 1U;
        memset(s_cal_sum, 0, sizeof s_cal_sum);
        memset(s_cal_sq, 0, sizeof s_cal_sq);
    }
    if (s_cal_n == 0U) {
        return;
    }
    if (--s_cal_n > 0U) {
        for (int i = 0; i < 3; i++) {
            s_cal_sum[i] += gyro_dps[i];
            s_cal_sq[i] += gyro_dps[i] * gyro_dps[i];
        }
        return;
    }
    float worst = 0.0f;
    for (int i = 0; i < 3; i++) {
        const float m = s_cal_sum[i] / (float)CAL_SAMPLES;
        const float v = s_cal_sq[i] / (float)CAL_SAMPLES - m * m;
        worst = v > worst ? v : worst;
        s_cal_mean[i] = m;
        s_mah.b[i] = m * DEG2RAD;
    }
    s_cal_std_dps = sqrtf(worst);
    s_cal_done = true;
    *bias_set = true;
}

static void ctrl_step(void)
{
    msg_sample_t m;
    imu_sample_t s;

    if (s_pending_set) {
        s_params = s_pending;
        s_param_rev = s_pending_rev;
        s_ekf.p = s_params.ekf;
        s_mah.p = s_params.mah;
        s_pending_set = false;
    }
    memset(&m, 0, sizeof m);
    m.type = MSG_SAMPLE;
    m.seq = s_seq++;
    m.param_rev = s_param_rev;

    if (!imu_read(&s)) {
        m.flags = SF_IMU_ERROR;
    } else {
        if (s_req_reset_ekf || !s_ekf_ready) {
            s_req_reset_ekf = false;
            ekf_ahrs_init(&s_ekf, &s_params.ekf, s.accel_g);
            s_ekf_ready = true;
            s_ekf_epoch++;
            m.flags |= SF_EKF_INIT;
        } else {
            ekf_ahrs_step(&s_ekf, s.gyro_dps, s.accel_g, DT);
        }

        bool bias_set = false;
        if (s_req_reset_mah || !s_mah_ready) {
            s_req_reset_mah = false;
            ahrs_reset(&s_mah, &s_params.mah, s.accel_g);
            s_mah_ready = true;
            s_mah_epoch++;
            m.flags |= SF_MAH_INIT;
        } else {
            mahony_cal_feed(s.gyro_dps, &bias_set);
            if (bias_set) {
                m.flags |= SF_MAH_BIAS_SET;
            } else {
                ahrs_update(&s_mah, s.gyro_dps, s.accel_g, DT, false);
            }
        }
        memcpy(m.accel_g, s.accel_g, sizeof m.accel_g);
        memcpy(m.gyro_dps, s.gyro_dps, sizeof m.gyro_dps);
        m.temp_c = s.temp_c;
    }

    m.flags |= (s_ekf.still ? SF_STILL : 0U) | (s_ekf.diag.accel_used ? SF_ACCEL_USED : 0U) |
               (s_ekf.diag.zupt_used ? SF_ZUPT_USED : 0U) | (s_mah.accel_used ? SF_MAH_ACCEL_USED : 0U);
    m.ekf_epoch = s_ekf_epoch;
    m.mah_epoch = s_mah_epoch;
    memcpy(m.ekf_q, s_ekf.q, sizeof m.ekf_q);
    memcpy(m.ekf_b, s_ekf.b, sizeof m.ekf_b);
    for (int i = 0; i < 3; i++) {
        m.ekf_sig_th[i] = sqrtf(fmaxf(s_ekf.P[i][i], 0.0f));
        m.ekf_sig_b[i] = sqrtf(fmaxf(s_ekf.P[3 + i][3 + i], 0.0f));
    }
    m.ekf_nis = s_ekf.diag.nis;
    memcpy(m.mah_q, s_mah.q, sizeof m.mah_q);
    memcpy(m.mah_b, s_mah.b, sizeof m.mah_b);
    m.isr_us = (uint16_t)ctrl_tick_last_us();
    m.overruns = (uint16_t)ctrl_tick_overruns();

    const uint32_t next = (s_q_head + 1U) % QUEUE_LEN;
    if (next == s_q_tail) {
        s_q_dropped++;
        return;
    }
    s_queue[s_q_head] = m;
    s_q_head = next;
}

static void send_text(const char *fmt, ...)
{
    uint8_t buf[1 + 120];
    va_list ap;

    buf[0] = MSG_TEXT;
    va_start(ap, fmt);
    const int n = vsnprintf((char *)&buf[1], sizeof buf - 1U, fmt, ap);
    va_end(ap);
    if (n > 0) {
        link_send(buf, 1U + (size_t)(n < (int)sizeof buf - 1 ? n : (int)sizeof buf - 2));
    }
}

static void send_hello(hw_version_t hw)
{
    msg_hello_t h;

    memset(&h, 0, sizeof h);
    h.type = MSG_HELLO;
    h.proto_version = BENCH_PROTO_VERSION;
    h.rate_hz = CTRL_HZ;
    h.param_count = PARAM_COUNT;
    h.hw_version = (uint8_t)hw;
    strncpy(h.name, "ahrs_bench", sizeof h.name);
    link_send(&h, sizeof h);
}

/* 主迴圈呼叫：rev 為這組參數的版本，p 為內容。 */
static void send_params(uint8_t rev, const bench_params_t *p)
{
    uint8_t buf[3 + PARAM_COUNT * sizeof(bench_param_t)];

    buf[0] = MSG_PARAMS;
    buf[1] = rev;
    buf[2] = PARAM_COUNT;
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        bench_param_t e;
        memset(&e, 0, sizeof e);
        strncpy(e.name, k_params[i].name, sizeof e.name);
        e.value = *param_ptr((bench_params_t *)p, i);
        memcpy(&buf[3 + i * sizeof e], &e, sizeof e);
    }
    while (!link_send(buf, sizeof buf)) {
    }
}

static bench_params_t current_params(uint8_t *rev)
{
    bench_params_t p;
    __disable_irq();
    p = s_pending_set ? s_pending : s_params;
    *rev = s_pending_set ? s_pending_rev : s_param_rev;
    __enable_irq();
    return p;
}

static void handle(const uint8_t *msg, size_t len, hw_version_t hw)
{
    if (msg[0] == MSG_CMD && len >= 2U) {
        switch (msg[1]) {
        case CMD_RESET_EKF:
            s_req_reset_ekf = true;
            break;
        case CMD_RESET_MAHONY:
            s_req_reset_mah = true;
            break;
        case CMD_RESET_BOTH:
            s_req_reset_ekf = s_req_reset_mah = true;
            break;
        case CMD_MAHONY_CAL:
            s_req_cal = true;
            send_text("mahony cal: keep still for 2 s");
            break;
        case CMD_HELLO: {
            uint8_t rev;
            const bench_params_t p = current_params(&rev);
            send_hello(hw);
            send_params(rev, &p);
            break;
        }
        default:
            send_text("unknown command %u", msg[1]);
            break;
        }
    } else if (msg[0] == MSG_SET_PARAMS && len >= 2U && msg[1] == PARAM_COUNT &&
               len == 2U + PARAM_COUNT * sizeof(float)) {
        uint8_t rev;
        bench_params_t p = current_params(&rev);
        for (size_t i = 0; i < PARAM_COUNT; i++) {
            memcpy(param_ptr(&p, i), &msg[2 + i * sizeof(float)], sizeof(float));
        }
        rev++;
        /* 先送出 PARAMS 再讓中斷套用，主機因此一定先知道新版本的內容，才收到用它的樣本。 */
        send_params(rev, &p);
        __disable_irq();
        s_pending = p;
        s_pending_rev = rev;
        s_pending_set = true;
        __enable_irq();
    } else {
        send_text("bad message 0x%02X len %u", msg[0], (unsigned)len);
    }
}

static void render(void)
{
    char line[24];
    float r, p, y;
    const link_stats_t st = link_stats();

    oled_clear();
    oled_text(0, 0, " ahrs_bench  921600");
    oled_invert_row(0);
    ekf_ahrs_euler(&s_ekf, &r, &p, &y);
    snprintf(line, sizeof line, "EKF %+6.1f%+6.1f", (double)r, (double)p);
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "    yaw %+7.1f", (double)y);
    oled_text(0, 2, line);
    ahrs_euler(&s_mah, &r, &p, &y);
    snprintf(line, sizeof line, "MAH %+6.1f%+6.1f", (double)r, (double)p);
    oled_text(0, 3, line);
    snprintf(line, sizeof line, "    yaw %+7.1f", (double)y);
    oled_text(0, 4, line);
    snprintf(line, sizeof line, "tx %lu q-drop %lu", (unsigned long)st.tx_frames, (unsigned long)s_q_dropped);
    oled_text(0, 5, line);
    snprintf(line, sizeof line, "rx %lu err %lu", (unsigned long)st.rx_frames, (unsigned long)st.rx_errors);
    oled_text(0, 6, line);
    oled_text(0, 7, "hold: reset filters");
    oled_flush();
}

int main(void)
{
    board_init();
    printf("\n==== ahrs_bench ====\n");

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
    printf("switching USART1 to %u baud binary protocol; run tools/ahrs_bench/server.py\n", BENCH_BAUD);
    link_init();
    send_hello(id.version);
    send_params(s_param_rev, &s_params);
    ctrl_tick_start(CTRL_HZ, ctrl_step);

    uint8_t msg[LINK_MAX_PAYLOAD];
    uint32_t last_oled = 0;
    for (;;) {
        const size_t n = link_poll(msg);
        if (n > 0U) {
            handle(msg, n, id.version);
        }
        while (s_q_tail != s_q_head) {
            if (!link_send(&s_queue[s_q_tail], sizeof(msg_sample_t))) {
                break;
            }
            s_q_tail = (s_q_tail + 1U) % QUEUE_LEN;
        }
        if (s_cal_done) {
            s_cal_done = false;
            send_text("mahony cal: bias %+.3f %+.3f %+.3f dps, max std %.3f dps%s", (double)s_cal_mean[0],
                      (double)s_cal_mean[1], (double)s_cal_mean[2], (double)s_cal_std_dps,
                      s_cal_std_dps > 0.3f ? " (moved? redo)" : "");
        }
        if (key_poll() == KEY_LONG) {
            s_req_reset_ekf = s_req_reset_mah = true;
            send_text("key: reset both filters");
        }
        const uint32_t now = HAL_GetTick();
        if (has_oled && now - last_oled >= OLED_PERIOD_MS) {
            last_oled = now;
            render();
        }
    }
}
