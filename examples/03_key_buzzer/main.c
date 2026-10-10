/*
 * 03 按鍵、使能開關與蜂鳴器。
 *   - 使用者按鍵（PC5）：在時限內按下並放開一次。
 *   - 使能開關（PE4）：在時限內切換到兩種狀態各一次。
 *   - 蜂鳴器（PD11）：響三聲，由使用者回答是否聽到；需插上蜂鳴器跳線帽。
 */
#include <stdio.h>

#include "board.h"
#include "console.h"
#include "report.h"

#define ACTION_TIMEOUT_MS 15000U
#define DEBOUNCE_MS       20U

/* 等待按鍵穩定地變成 pressed 狀態；逾時回傳 false。 */
static bool wait_key(bool pressed, uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    uint32_t stable_since = HAL_GetTick();

    while (HAL_GetTick() - start < timeout_ms) {
        if (user_key_pressed() != pressed) {
            stable_since = HAL_GetTick();
        } else if (HAL_GetTick() - stable_since >= DEBOUNCE_MS) {
            return true;
        }
    }
    return false;
}

static void test_user_key(void)
{
    if (user_key_pressed()) {
        report_check(false, "user key", "reads pressed while idle (stuck low?)");
        return;
    }
    printf("  press and release the user key (PC5) within %lus\n",
           (unsigned long)(ACTION_TIMEOUT_MS / 1000U));
    const uint32_t t0 = HAL_GetTick();
    if (!wait_key(true, ACTION_TIMEOUT_MS)) {
        report_check(false, "user key", "no press detected");
        return;
    }
    const uint32_t t_press = HAL_GetTick();
    led_set(false, false, true);
    if (!wait_key(false, ACTION_TIMEOUT_MS)) {
        report_check(false, "user key", "pressed but never released");
        return;
    }
    led_set(false, false, false);
    report_check(true, "user key", "pressed after %lu ms, held %lu ms",
                 (unsigned long)(t_press - t0), (unsigned long)(HAL_GetTick() - t_press));
}

static void test_enable_switch(void)
{
    const bool initial = enable_switch_high();
    bool seen_high = initial;
    bool seen_low = !initial;
    const uint32_t start = HAL_GetTick();

    printf("  enable switch (PE4) reads %s; toggle it both ways within %lus\n",
           initial ? "HIGH" : "LOW", (unsigned long)(ACTION_TIMEOUT_MS / 1000U));
    while (!(seen_high && seen_low) && HAL_GetTick() - start < ACTION_TIMEOUT_MS) {
        const bool level = enable_switch_high();
        seen_high |= level;
        seen_low |= !level;
        led_set(false, level, false);
        HAL_Delay(DEBOUNCE_MS);
    }
    led_set(false, false, false);
    report_check(seen_high && seen_low, "enable switch", "seen HIGH=%s LOW=%s",
                 seen_high ? "yes" : "no", seen_low ? "yes" : "no");
}

static void test_buzzer(void)
{
    printf("  buzzer (PD11) beeps 3 times; the buzzer jumper must be fitted\n");
    HAL_Delay(500);
    for (int i = 0; i < 3; i++) {
        buzzer_set(true);
        HAL_Delay(150);
        buzzer_set(false);
        HAL_Delay(250);
    }
    const answer_t a = console_ask("did you hear 3 beeps?", ACTION_TIMEOUT_MS);
    if (a == ANSWER_TIMEOUT) {
        report_skip("buzzer", "no answer");
    } else {
        report_check(a == ANSWER_YES, "buzzer", "3 x 150 ms");
    }
}

int main(void)
{
    board_init();
    report_begin("03_key_buzzer");

    test_user_key();
    test_enable_switch();
    test_buzzer();

    report_end();
    report_halt();
}
