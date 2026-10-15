/*
 * 31 編碼器：用手轉動驅動輪，確認編碼器通道、方向與每圈計數。
 *   1. 靜止 1 秒：四路計數都不應變動（排除浮接或雜訊）。
 *   2. 左、右驅動輪各往「朝驅動端」的方向用手轉一整圈，轉完按使用者按鍵。
 *      回報是哪一路編碼器、正負號、計數，並由計數反推編碼器線數。
 *   3. 之後以 5 Hz 持續輸出四路計數。
 * 馬達不通電，D50A 為脫機（IN 腳有下拉），輪子可以用手轉動。
 * 在輪子與底盤上各做一個記號，比較容易轉剛好一圈。
 */
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "encoder.h"
#include "report.h"

#define ROTATE_TIMEOUT_MS   60000U
#define DEBOUNCE_MS         20U
#define REV_TOLERANCE       0.10f   /* 手轉一圈的誤差容許 */
#define CROSSTALK_RATIO     0.05f   /* 其他通道的計數須小於主要通道的 5% */

typedef struct {
    encoder_id_t channel;
    int32_t counts;
} wheel_result_t;

static bool wait_key_state(bool pressed, uint32_t timeout_ms, int32_t counts[ENC_COUNT])
{
    const uint32_t start = HAL_GetTick();
    uint32_t stable_since = HAL_GetTick();

    while (HAL_GetTick() - start < timeout_ms) {
        encoder_poll(counts);
        if (user_key_pressed() != pressed) {
            stable_since = HAL_GetTick();
        } else if (HAL_GetTick() - stable_since >= DEBOUNCE_MS) {
            return true;
        }
        HAL_Delay(5);
    }
    return false;
}

static void test_idle(void)
{
    int32_t counts[ENC_COUNT];

    encoder_reset();
    HAL_Delay(1000);
    encoder_poll(counts);
    bool ok = true;
    for (int i = 0; i < ENC_COUNT; i++) {
        ok &= abs(counts[i]) <= 2;
    }
    report_check(ok, "idle", "1 s without motion: A %ld  B %ld  C %ld  D %ld",
                 (long)counts[0], (long)counts[1], (long)counts[2], (long)counts[3]);
}

/* 回傳轉動的是哪一路；沒有偵測到明確單一通道時 channel = ENC_COUNT。 */
static wheel_result_t measure_wheel(const char *side)
{
    wheel_result_t result = {ENC_COUNT, 0};
    int32_t counts[ENC_COUNT] = {0};

    printf("  rotate the %s drive wheel ONE full turn toward the drive end, then press the user key\n",
           side);
    encoder_reset();
    if (!wait_key_state(true, ROTATE_TIMEOUT_MS, counts)) {
        report_skip(side, "no key press");
        return result;
    }
    wait_key_state(false, ROTATE_TIMEOUT_MS, NULL);

    int best = 0;
    for (int i = 1; i < ENC_COUNT; i++) {
        if (abs(counts[i]) > abs(counts[best])) {
            best = i;
        }
    }
    bool single = abs(counts[best]) > 100;
    for (int i = 0; i < ENC_COUNT; i++) {
        if (i != best && (float)abs(counts[i]) > CROSSTALK_RATIO * (float)abs(counts[best])) {
            single = false;
        }
    }
    printf("  counts: A %ld  B %ld  C %ld  D %ld\n",
           (long)counts[0], (long)counts[1], (long)counts[2], (long)counts[3]);
    if (!report_check(single, side, "one encoder should move clearly, the others stay still")) {
        return result;
    }

    result.channel = (encoder_id_t)best;
    result.counts = counts[best];
    const float revs = (float)abs(result.counts) / (float)COUNTS_PER_WHEEL_REV;
    const float lines = (float)abs(result.counts) / (4.0f * (float)MOTOR_GEAR_RATIO);
    printf("  %s wheel = encoder %s, %s counts when rolling toward the drive end\n",
           side, encoder_name(result.channel), result.counts > 0 ? "positive" : "negative");
    report_check(revs > 1.0f - REV_TOLERANCE && revs < 1.0f + REV_TOLERANCE,
                 side, "%ld counts = %.2f rev at %u counts/rev (implies %.0f-line encoder at 1:%u)",
                 (long)result.counts, (double)revs, COUNTS_PER_WHEEL_REV,
                 (double)lines, MOTOR_GEAR_RATIO);
    return result;
}

static void stream(void)
{
    int32_t counts[ENC_COUNT];

    printf("  streaming at 5 Hz: A B C D (counts)\n");
    encoder_reset();
    for (;;) {
        for (int i = 0; i < 40; i++) {
            encoder_poll(counts);
            HAL_Delay(5);
        }
        printf("  %9ld %9ld %9ld %9ld\n",
               (long)counts[0], (long)counts[1], (long)counts[2], (long)counts[3]);
    }
}

int main(void)
{
    board_init();
    report_begin("31_encoder");
    encoder_init();

    test_idle();
    const wheel_result_t left = measure_wheel("left");
    const wheel_result_t right = measure_wheel("right");

    if (left.channel != ENC_COUNT && right.channel != ENC_COUNT) {
        report_check(left.channel != right.channel, "left/right channels",
                     "left = %s, right = %s", encoder_name(left.channel), encoder_name(right.channel));
        report_check((left.channel == ENC_A || left.channel == ENC_B) &&
                     (right.channel == ENC_A || right.channel == ENC_B),
                     "drive wheels on A/B", "the chassis motors are expected on encoder ports A and B");
    }

    report_end();
    stream();
}
