/*
 * 32 馬達開環：馬達 A、B 各以兩種 IN 方向低速運轉，用編碼器確認是否轉動、
 * 帶動哪一路編碼器、反向時編碼器正負號是否相反，並請使用者指出是左輪或右輪。
 *
 * 安全：
 *   - 車子必須架高，兩個驅動輪離地；程式會先詢問。
 *   - 使能開關（PE4）必須為高才會轉；運轉中切到低、或按下使用者按鍵，立即停止。
 *   - 佔空比上限 25%，加減速 1 秒緩升緩降（D50A 手冊要求反轉前先降到 0）。
 */
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"
#include "board.h"
#include "console.h"
#include "encoder.h"
#include "motor.h"
#include "report.h"

#define TEST_DUTY          0.25f
#define RAMP_MS            1000U
#define HOLD_MS            1500U
#define COAST_MS           800U
#define STEP_MS            10U
#define MIN_VBAT           7.0f
#define MIN_MOVE_REV       0.05f    /* 保持期間至少轉 0.05 圈才算有轉 */
#define CROSSTALK_RATIO    0.05f
#define SYMMETRY_TOLERANCE 0.30f    /* 正反轉速度差距 */
#define ANSWER_TIMEOUT_MS  20000U

typedef struct {
    int32_t hold_counts[ENC_COUNT];
    bool aborted;
} run_result_t;

static bool must_stop(void)
{
    return !enable_switch_high() || user_key_pressed();
}

/* 以 duty 運轉一次：緩升、保持並量測、緩降、脫機。 */
static run_result_t run_motor(motor_id_t motor, float duty)
{
    run_result_t r = {{0}, false};
    int32_t start[ENC_COUNT], now[ENC_COUNT];

    for (uint32_t t = 0; t <= RAMP_MS; t += STEP_MS) {
        if (must_stop()) {
            goto abort;
        }
        motor_set(motor, duty * (float)t / (float)RAMP_MS);
        encoder_poll(NULL);
        HAL_Delay(STEP_MS);
    }
    encoder_poll(start);
    for (uint32_t t = 0; t < HOLD_MS; t += STEP_MS) {
        if (must_stop()) {
            goto abort;
        }
        encoder_poll(NULL);
        HAL_Delay(STEP_MS);
    }
    encoder_poll(now);
    for (int i = 0; i < ENC_COUNT; i++) {
        r.hold_counts[i] = now[i] - start[i];
    }
    for (uint32_t t = RAMP_MS; t > 0; t -= STEP_MS) {
        if (must_stop()) {
            goto abort;
        }
        motor_set(motor, duty * (float)t / (float)RAMP_MS);
        encoder_poll(NULL);
        HAL_Delay(STEP_MS);
    }
    motor_set(motor, 0.0f);
    for (uint32_t t = 0; t < COAST_MS; t += STEP_MS) {
        encoder_poll(NULL);
        HAL_Delay(STEP_MS);
    }
    return r;

abort:
    motor_stop_all();
    r.aborted = true;
    return r;
}

static int dominant(const int32_t c[ENC_COUNT])
{
    int best = 0;
    for (int i = 1; i < ENC_COUNT; i++) {
        if (abs(c[i]) > abs(c[best])) {
            best = i;
        }
    }
    return best;
}

static float rpm(int32_t counts)
{
    return (float)counts / (float)COUNTS_PER_WHEEL_REV / ((float)HOLD_MS / 60000.0f);
}

static bool preflight(void)
{
    battery_reading_t bat = {0};

    battery_init();
    if (!report_check(battery_measure(&bat) && bat.vbat > MIN_VBAT, "battery",
                      "%.2f V (need > %.1f V for the D50A)", (double)bat.vbat, (double)MIN_VBAT)) {
        return false;
    }
    if (console_ask("are both drive wheels OFF the ground?", ANSWER_TIMEOUT_MS) != ANSWER_YES) {
        report_skip("motors", "lift the car so the drive wheels can spin freely, then rerun");
        return false;
    }
    if (!enable_switch_high()) {
        printf("  turn the enable switch ON (PE4 high) within 15 s\n");
        const uint32_t start = HAL_GetTick();
        while (!enable_switch_high() && HAL_GetTick() - start < 15000U) {
        }
    }
    if (!report_check(enable_switch_high(), "enable switch", "PE4 high")) {
        return false;
    }
    printf("  motors will turn now. press the user key or switch enable OFF to stop at any time\n");
    HAL_Delay(1500);
    return true;
}

static void test_motor(motor_id_t motor)
{
    const char *name = motor == MOTOR_A ? "motor A" : "motor B";
    const encoder_id_t expected = motor == MOTOR_A ? ENC_A : ENC_B;

    printf("  %s: IN1=H/IN2=L at %.0f%%\n", name, (double)(TEST_DUTY * 100.0f));
    led_set(false, false, true);
    const run_result_t fwd = run_motor(motor, TEST_DUTY);
    if (fwd.aborted) {
        report_check(false, name, "aborted by key or enable switch");
        return;
    }
    const answer_t side = console_ask("was that the LEFT drive wheel? (n = right)", ANSWER_TIMEOUT_MS);

    printf("  %s: IN1=L/IN2=H at %.0f%%\n", name, (double)(TEST_DUTY * 100.0f));
    const run_result_t rev = run_motor(motor, -TEST_DUTY);
    led_set(false, false, false);
    if (rev.aborted) {
        report_check(false, name, "aborted by key or enable switch");
        return;
    }

    const int ch = dominant(fwd.hold_counts);
    const int32_t f = fwd.hold_counts[ch];
    const int32_t r = rev.hold_counts[ch];
    printf("  hold counts IN1=H: A %ld B %ld C %ld D %ld | IN1=L: A %ld B %ld C %ld D %ld\n",
           (long)fwd.hold_counts[0], (long)fwd.hold_counts[1],
           (long)fwd.hold_counts[2], (long)fwd.hold_counts[3],
           (long)rev.hold_counts[0], (long)rev.hold_counts[1],
           (long)rev.hold_counts[2], (long)rev.hold_counts[3]);

    if (!report_check((float)abs(f) > MIN_MOVE_REV * (float)COUNTS_PER_WHEEL_REV, name,
                      "turns: %.1f rpm at IN1=H (wheel rpm, %u counts/rev)",
                      (double)rpm(f), COUNTS_PER_WHEEL_REV)) {
        printf("  no encoder moved: check the D50A power, the P2 ribbon cable and the encoder cable\n");
        return;
    }
    bool single = true;
    for (int i = 0; i < ENC_COUNT; i++) {
        if (i != ch && (float)abs(fwd.hold_counts[i]) > CROSSTALK_RATIO * (float)abs(f)) {
            single = false;
        }
    }
    report_check(single, name, "drives only encoder %s", encoder_name((encoder_id_t)ch));
    report_check(ch == (int)expected, name, "encoder %s (expected %s: motor and encoder port letters match)",
                 encoder_name((encoder_id_t)ch), encoder_name(expected));
    const float asym = (float)abs(abs(f) - abs(r)) / (float)abs(f);
    report_check((f > 0) != (r > 0) && asym < SYMMETRY_TOLERANCE, name,
                 "reverse: %+.1f rpm vs %+.1f rpm (opposite sign, within 30%%)",
                 (double)rpm(f), (double)rpm(r));
    if (side != ANSWER_TIMEOUT) {
        printf("  => %s is the %s wheel; IN1=H gives %s counts on encoder %s\n",
               name, side == ANSWER_YES ? "LEFT" : "RIGHT", f > 0 ? "positive" : "negative",
               encoder_name((encoder_id_t)ch));
    }
}

int main(void)
{
    board_init();
    motor_init();          /* 一開機就把 IN 腳拉低，確保脫機 */
    report_begin("32_motor");
    encoder_init();

    if (preflight()) {
        test_motor(MOTOR_A);
        test_motor(MOTOR_B);
    }
    motor_stop_all();
    printf("  compare with 31_encoder: the encoder sign when rolling toward the drive end\n"
           "  tells which IN pattern drives each wheel forward\n");
    report_end();
    report_halt();
}
