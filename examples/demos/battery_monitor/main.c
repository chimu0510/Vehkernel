/*
 * 電池監控：持續量測電池電壓，依電量等級以 LED、蜂鳴器與 OLED 警告（門檻見 battery_monitor.h）。
 *
 *   等級       LED              蜂鳴器
 *   OK         綠燈恆亮         —
 *   LOW        黃燈 1 Hz 閃     每秒嗶一聲（原廠代碼的低電量提示）
 *   CRITICAL   紅燈 4 Hz 閃     每秒連嗶三聲
 *   NONE       藍燈恆亮         —（只有 USB 供電）
 *
 * 長按使用者按鍵：靜音，直到等級再次改變。
 * 電池電壓不容易調低，所以可以在主控台輸入模擬電壓驗證門檻與遲滯：
 *   sim 19.5   以 19.5 V 取代 ADC 量測
 *   sim off    回到 ADC 量測
 * 等級改變時印一行，另外每秒印一行電壓。
 *
 * 依賴（皆已通過功能驗證）：11_led、12_key_buzzer、21_oled、23_battery_adc。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "battery_monitor.h"
#include "board.h"
#include "board_id.h"
#include "console.h"
#include "key.h"
#include "oled.h"

#define LOOP_MS        10U
#define OLED_PERIOD_MS 200U
#define LOG_PERIOD_MS  1000U

static bool s_muted;
static float s_min_v = 1e9f, s_max_v;
static char s_line[16];
static size_t s_line_len;

/* 依等級與時間決定 LED 與蜂鳴器；period 內的相位以 HAL_GetTick() 計算。 */
static void indicate(batt_level_t level, uint32_t now)
{
    const uint32_t t = now % 1000U;
    bool beep = false;

    switch (level) {
    case BATT_OK:
        led_set(false, true, false);
        break;
    case BATT_LOW:
        led_set(t < 500U, t < 500U, false);
        beep = t < 200U;
        break;
    case BATT_CRITICAL:
        led_set(t % 250U < 125U, false, false);
        beep = t < 500U && t % 170U < 100U;
        break;
    case BATT_NONE:
        led_set(false, false, true);
        break;
    }
    buzzer_set(beep && !s_muted);
}

static void draw_bar(int y, int percent)
{
    oled_rect(4, y, OLED_WIDTH - 8, 7, true);
    const int fill = (OLED_WIDTH - 12) * percent / 100;
    for (int x = 0; x < fill; x++) {
        for (int j = 2; j < 5; j++) {
            oled_pixel(6 + x, y + j, true);
        }
    }
}

static void render(const batt_status_t *st)
{
    char line[24];

    oled_clear();
    oled_text(0, 0, " Battery");
    oled_invert_row(0);
    snprintf(line, sizeof line, "Vbat  %6.2f V%s", (double)st->vbat_filtered, st->simulated ? " SIM" : "");
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "Level %s", batt_level_name(st->level));
    oled_text(0, 2, line);
    if (st->level == BATT_CRITICAL || st->level == BATT_LOW) {
        oled_invert_row(2);
    }
    snprintf(line, sizeof line, "Charge %3d %%", st->percent);
    oled_text(0, 3, line);
    draw_bar(4 * OLED_CHAR_H, st->level == BATT_NONE ? 0 : st->percent);
    if (s_max_v > 0.0f) {
        snprintf(line, sizeof line, "min %5.2f max %5.2f", (double)s_min_v, (double)s_max_v);
        oled_text(0, 5, line);
    }
    oled_text(0, 7, s_muted ? "muted (hold: unmute)" : "hold: mute");
    oled_flush();
}

/* 非阻塞讀取主控台的一行指令；以輪詢收字元，貼上一長串可能掉字，請用手打。 */
static void poll_console(void)
{
    const int c = console_getc(0);
    if (c < 0) {
        return;
    }
    if (c != '\r' && c != '\n') {
        if (s_line_len < sizeof s_line - 1U) {
            const char ch = (char)c;
            s_line[s_line_len++] = ch;
            console_write((const uint8_t *)&ch, 1);   /* 回顯 */
        }
        return;
    }
    s_line[s_line_len] = '\0';
    s_line_len = 0;
    printf("\n");
    if (strncmp(s_line, "sim ", 4) == 0) {
        if (strcmp(s_line + 4, "off") == 0) {
            battery_monitor_simulate(0.0f);
            printf("simulation off\n");
        } else {
            const float v = strtof(s_line + 4, NULL);
            battery_monitor_simulate(v);
            printf("simulating %.2f V\n", (double)v);
        }
    } else if (s_line[0] != '\0') {
        printf("commands: sim <volts> | sim off\n");
    }
}

int main(void)
{
    board_init();
    printf("\n==== battery_monitor ====\n");
    printf("thresholds: low < %.1f V (clear >= %.1f), critical < %.1f V (clear >= %.1f)\n",
           (double)BATT_LOW_ON_V, (double)BATT_LOW_OFF_V,
           (double)BATT_CRITICAL_ON_V, (double)BATT_CRITICAL_OFF_V);
    printf("commands: sim <volts> | sim off\n");

    const board_id_t id = board_id_detect();
    const bool has_oled = id.version == HW_V1_0 || id.version == HW_V1_1;
    if (has_oled) {
        oled_init();
    }
    battery_monitor_init();
    const batt_status_t *st = battery_monitor_status();
    printf("level %s at %.2f V\n", batt_level_name(st->level), (double)st->vbat);

    uint32_t last_oled = 0, last_log = 0;
    for (;;) {
        const uint32_t now = HAL_GetTick();
        if (battery_monitor_update()) {
            s_muted = false;
            printf("level -> %s at %.2f V (filtered)\n", batt_level_name(st->level), (double)st->vbat_filtered);
        }
        if (st->level != BATT_NONE && !st->simulated) {
            if (st->vbat_filtered < s_min_v) {
                s_min_v = st->vbat_filtered;
            }
            if (st->vbat_filtered > s_max_v) {
                s_max_v = st->vbat_filtered;
            }
        }
        if (key_poll() == KEY_LONG) {
            s_muted = !s_muted;
            printf("%s\n", s_muted ? "muted" : "unmuted");
        }
        poll_console();
        indicate(st->level, now);
        if (has_oled && now - last_oled >= OLED_PERIOD_MS) {
            last_oled = now;
            render(st);
        }
        if (now - last_log >= LOG_PERIOD_MS) {
            last_log = now;
            printf("vbat %.2f V, filtered %.2f V, %d %%, %s%s\n", (double)st->vbat,
                   (double)st->vbat_filtered, st->percent, batt_level_name(st->level),
                   st->simulated ? " (sim)" : "");
        }
        HAL_Delay(LOOP_MS);
    }
}
