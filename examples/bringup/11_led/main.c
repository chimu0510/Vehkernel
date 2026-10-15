/*
 * 11 LED：確認 RGB LED 的腳位對應與點亮電位，再確認各顏色。
 *   1. 腳位辨識：每次只把一支 LED 腳拉高（其餘拉低），由使用者回答看到的顏色。
 *      高電位點亮時每次只會亮一色；低電位點亮時會亮另外兩色的混色。
 *      由此推算極性與每支腳的顏色，並與 board.h 比對。
 *   2. 顏色確認：依 board.h 點亮紅、綠、藍、白、全滅，由使用者回答 y/n。
 * 同一組腳位也接到 RGB 燈帶接口，有接燈帶時燈帶會一起變色。
 */
#include <stdio.h>

#include "board.h"
#include "console.h"
#include "report.h"

#define ANSWER_TIMEOUT_MS 30000U

enum { RED = 1, GREEN = 2, BLUE = 4 };

typedef struct {
    const char *name;     /* board.h 的名稱 */
    GPIO_TypeDef *port;
    uint16_t pin;
    unsigned expected;    /* board.h 宣稱的顏色 */
} led_pin_t;

static const led_pin_t k_pins[] = {
    {"LED_R", LED_R_PORT, LED_R_PIN, RED},
    {"LED_G",  LED_G_PORT, LED_G_PIN, GREEN},
    {"LED_B", LED_B_PORT, LED_B_PIN, BLUE},
};
#define PIN_COUNT (sizeof k_pins / sizeof k_pins[0])

typedef struct {
    const char *name;
    bool r, g, b;
} color_t;

static const color_t k_colors[] = {
    {"red",   true,  false, false},
    {"green", false, true,  false},
    {"blue",  false, false, true},
    {"white", true,  true,  true},
    {"off",   false, false, false},
};

static const char *color_name(unsigned mask)
{
    static const char *const names[] = {
        "off", "red", "green", "yellow", "blue", "magenta", "cyan", "white",
    };
    return names[mask & 7U];
}

static unsigned bit_count(unsigned v)
{
    unsigned n = 0;
    for (; v; v &= v - 1U) {
        n++;
    }
    return n;
}

/* 讀一個顏色代號，回傳 RGB 位元遮罩；逾時回傳 -1。 */
static int ask_color(void)
{
    const uint32_t start = HAL_GetTick();

    console_flush_input();
    printf("  ? color: r=red g=green b=blue y=yellow c=cyan m=magenta w=white k=off > ");
    while (HAL_GetTick() - start < ANSWER_TIMEOUT_MS) {
        const int c = console_getc(10);
        int mask = -1;
        switch (c) {
        case 'r': mask = RED; break;
        case 'g': mask = GREEN; break;
        case 'b': mask = BLUE; break;
        case 'y': mask = RED | GREEN; break;
        case 'c': mask = GREEN | BLUE; break;
        case 'm': mask = RED | BLUE; break;
        case 'w': mask = RED | GREEN | BLUE; break;
        case 'k': mask = 0; break;
        default: break;
        }
        if (mask >= 0) {
            printf("%s\n", color_name((unsigned)mask));
            return mask;
        }
    }
    printf("(timeout)\n");
    return -1;
}

static void drive_only_high(unsigned index)
{
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        HAL_GPIO_WritePin(k_pins[i].port, k_pins[i].pin, i == index ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/* 回傳 board.h 是否正確（極性與三支腳的顏色都對）。 */
static bool identify_pins(void)
{
    unsigned seen[PIN_COUNT];

    printf("  step 1: one LED pin is driven HIGH at a time; type the color you see\n");
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        drive_only_high(i);
        printf("  %s high\n", k_pins[i].name);
        const int mask = ask_color();
        if (mask < 0) {
            report_skip("pin identification", "no answer");
            return false;
        }
        seen[i] = (unsigned)mask;
    }

    /* 高電位點亮：每支腳各亮一個不同的單色；低電位點亮：每次亮另外兩色，缺的那色才是該腳。 */
    bool active_high = true, active_low = true;
    unsigned all_high = 0, all_low = 0;
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        active_high &= bit_count(seen[i]) == 1U;
        active_low &= bit_count(seen[i]) == 2U;
        all_high |= seen[i];
        all_low |= ~seen[i] & 7U;
    }
    active_high &= all_high == 7U;
    active_low &= all_low == 7U;
    if (!report_check(active_high || active_low, "pin answers",
                      "%s / %s / %s (must be one LED color per pin)",
                      color_name(seen[0]), color_name(seen[1]), color_name(seen[2]))) {
        return false;
    }

    const bool polarity_ok = active_high == (LED_ON_LEVEL == GPIO_PIN_SET);
    report_check(polarity_ok, "led polarity", "active-%s (board.h LED_ON_LEVEL is GPIO_PIN_%s)",
                 active_high ? "high" : "low", LED_ON_LEVEL == GPIO_PIN_SET ? "SET" : "RESET");

    bool mapping_ok = true;
    for (unsigned i = 0; i < PIN_COUNT; i++) {
        const unsigned actual = active_high ? seen[i] : (~seen[i] & 7U);
        const bool ok = actual == k_pins[i].expected;
        report_check(ok, k_pins[i].name, "drives %s (board.h says %s)",
                     color_name(actual), color_name(k_pins[i].expected));
        mapping_ok &= ok;
    }
    return polarity_ok && mapping_ok;
}

static void confirm_colors(void)
{
    printf("  step 2: board.h colors; answer y/n\n");
    for (unsigned i = 0; i < sizeof k_colors / sizeof k_colors[0]; i++) {
        const color_t *c = &k_colors[i];
        char question[48];

        led_set(c->r, c->g, c->b);
        snprintf(question, sizeof question, "is the LED %s?", c->name);
        const answer_t a = console_ask(question, ANSWER_TIMEOUT_MS);
        if (a == ANSWER_TIMEOUT) {
            report_skip(c->name, "no answer");
        } else {
            report_check(a == ANSWER_YES, c->name, "R%d G%d B%d", c->r, c->g, c->b);
        }
    }
}

int main(void)
{
    board_init();
    report_begin("11_led");

    if (identify_pins()) {
        confirm_colors();
    } else {
        report_skip("colors", "fix board.h LED pins/LED_ON_LEVEL with the results above first");
    }

    led_set(false, false, false);
    const bool ok = report_end();

    /* 結束後以結果顏色慢閃，方便沒接主控台時也能看出結果。 */
    for (;;) {
        led_set(!ok, ok, false);
        HAL_Delay(500);
        led_set(false, false, false);
        HAL_Delay(500);
    }
}
