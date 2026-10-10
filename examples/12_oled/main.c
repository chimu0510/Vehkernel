/*
 * 12 OLED：0.96 吋 SSD1306（GPIO 模擬 SPI，只寫不讀），由使用者看螢幕回答 y/n。
 *   1. 全亮（0xA5，忽略 RAM）：確認接線、重設、電荷泵與顯示開啟。
 *   2. 邊框與文字：確認 128×64 位址對齊（SH1106 等 132 欄的控制器會缺邊或錯位）、
 *      字型與顯示方向。
 *   3. 反相與關閉顯示：確認命令有作用。
 *   4. 量測整個畫面更新的時間。
 * 之後持續在螢幕上顯示開機時間與使用者按鍵狀態。
 * 腳位只適用 V1.0/V1.1；V1.2 的 OLED 改接其他腳，偵測到時直接結束。
 */
#include <stdio.h>

#include "board.h"
#include "board_id.h"
#include "console.h"
#include "oled.h"
#include "report.h"

#define ANSWER_TIMEOUT_MS  30000U
#define MAX_FLUSH_MS       20U     /* 至少要能 50 Hz 更新整個畫面 */

/* 回傳是否答 yes；逾時記為 SKIP。 */
static bool ask_check(const char *name, const char *question, const char *detail)
{
    const answer_t a = console_ask(question, ANSWER_TIMEOUT_MS);
    if (a == ANSWER_TIMEOUT) {
        report_skip(name, "no answer");
        return false;
    }
    return report_check(a == ANSWER_YES, name, "%s", detail);
}

static void draw_test_pattern(void)
{
    oled_clear();
    oled_rect(0, 0, OLED_WIDTH, OLED_HEIGHT, true);
    oled_text(1, 1, "C50C  12_oled");
    oled_text(1, 3, "0123456789 +-=:.%");
    oled_text(1, 4, "ABCDEFGHIJKLMNOPQRS");
    oled_text(1, 5, "abcdefghijklmnopqrs");
    oled_flush();
}

static void stream(void)
{
    char line[24];

    printf("  the screen now shows uptime and the user key state\n");
    for (;;) {
        const uint32_t ms = HAL_GetTick();
        oled_clear();
        oled_rect(0, 0, OLED_WIDTH, OLED_HEIGHT, true);
        oled_text(1, 1, "C50C  12_oled");
        snprintf(line, sizeof line, "up  %5lu.%01lu s",
                 (unsigned long)(ms / 1000U), (unsigned long)(ms / 100U % 10U));
        oled_text(1, 3, line);
        snprintf(line, sizeof line, "key %s", user_key_pressed() ? "PRESSED" : "-");
        oled_text(1, 4, line);
        oled_flush();
        HAL_Delay(100);
    }
}

int main(void)
{
    board_init();
    report_begin("12_oled");

    const board_id_t id = board_id_detect();
    if (!report_check(id.version == HW_V1_0 || id.version == HW_V1_1, "hardware version",
                      "%s; board.h OLED pins are for V1.0/V1.1", board_version_name(id.version))) {
        report_end();
        report_halt();
    }

    oled_init();
    printf("  OLED on SCL PD7, SDA PD6, RST PD5, DC PD4\n");

    oled_all_on(true);
    if (!ask_check("all pixels", "is the WHOLE screen lit?",
                   "0xA5 entire display on (wiring, reset, charge pump)")) {
        printf("  check that the OLED module is plugged in the right way round (GND/3V3/PD7/PD6/PD5/PD4)\n");
        oled_all_on(false);
        report_end();
        report_halt();
    }
    oled_all_on(false);

    draw_test_pattern();
    ask_check("frame", "is there a complete 1-pixel frame along ALL FOUR edges?",
              "128x64 addressing; a missing edge or shifted image means a 132-column SH1106");
    ask_check("text", "is the text upright, readable and not mirrored?",
              "5x7 font, segment remap 0xA1 + COM scan 0xC8");

    oled_invert(true);
    ask_check("invert", "is it now inverted (lit background, dark text)?", "0xA7 inverse display");
    oled_invert(false);

    oled_display_on(false);
    ask_check("display off", "is the screen completely dark now?", "0xAE display off");
    oled_display_on(true);

    const uint32_t t0 = DWT->CYCCNT;
    oled_flush();
    const uint32_t flush_us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
    report_check(flush_us < MAX_FLUSH_MS * 1000U, "refresh time",
                 "full 1 KB frame in %lu us (limit %u ms)", (unsigned long)flush_us, MAX_FLUSH_MS);

    report_end();
    stream();
}
