/*
 * OLED 菜單：以使用者按鍵操作 OLED 上的菜單。
 *   單擊 = 下一項、長按 = 進入／確認、雙擊 = 返回（手勢見 key.h）。
 *
 *   Status   電池電壓、開機時間、車頭方向，以及左右驅動輪轉了幾圈；長按歸零。
 *            左右與正負號依開機時載入的車頭方向換算：前進為正，全向輪端在前時
 *            左右對調並反號，這就是 ADR-0001 說的「映射層切換左右通道並反轉方向」。
 *   Heading  車頭方向：長按把選取的值存進 Flash，重開機才生效（ADR-0001 不支援運行中切換）。
 *   About    硬體版本、IMU 型號、MCU UID 與建置日期。
 *
 * 依賴（皆已通過功能驗證）：12_key_buzzer、13_flash_params、21_oled、23_battery_adc、31_encoder。
 * 按鍵事件與存檔結果也會印到主控台。
 */
#include <stdio.h>

#include "battery.h"
#include "board.h"
#include "board_id.h"
#include "encoder.h"
#include "key.h"
#include "oled.h"
#include "settings.h"

#define LOOP_MS          5U
#define STATUS_PERIOD_MS 200U
#define BATTERY_EVERY    5U      /* 每 5 次狀態更新量一次電池，約 1 s */

typedef enum {
    SCR_MAIN,
    SCR_STATUS,
    SCR_HEADING,
    SCR_ABOUT,
} screen_t;

static const char *const k_main_items[] = {"Status", "Heading", "About"};
#define MAIN_COUNT 3

static const char *const k_heading_labels[] = {"Drive end front", "Omni end front"};

static screen_t s_screen = SCR_MAIN;
static int s_cursor;                 /* 目前畫面的選取項目 */
static settings_t s_boot;            /* 開機時載入、本次開機實際使用的設定 */
static settings_t s_saved;           /* Flash 中的設定（存檔後可能與 s_boot 不同） */
static const char *s_message = "";   /* Heading 畫面的狀態列 */
static board_id_t s_id;
static float s_vbat;
static int32_t s_counts[ENC_COUNT];
static int32_t s_zero[ENC_COUNT];

/* 驅動端在前時往驅動端為正；全向輪端在前時左右對調、往全向輪端為正。 */
static void wheel_revs(float *left, float *right)
{
    const float l = (float)(WHEEL_LEFT_ENC_SIGN * (s_counts[ENC_A] - s_zero[ENC_A])) /
                    (float)COUNTS_PER_WHEEL_REV;
    const float r = (float)(WHEEL_RIGHT_ENC_SIGN * (s_counts[ENC_B] - s_zero[ENC_B])) /
                    (float)COUNTS_PER_WHEEL_REV;
    if (s_boot.heading == HEADING_DRIVE_END_FRONT) {
        *left = l;
        *right = r;
    } else {
        *left = -r;
        *right = -l;
    }
}

static void title(const char *s)
{
    oled_text(0, 0, s);
    oled_invert_row(0);
}

static void draw_main(void)
{
    title(" C50C menu");
    for (int i = 0; i < MAIN_COUNT; i++) {
        char line[24];
        snprintf(line, sizeof line, "%s %s", i == s_cursor ? ">" : " ", k_main_items[i]);
        oled_text(0, 2 + i, line);
        if (i == s_cursor) {
            oled_invert_row(2 + i);
        }
    }
    oled_text(0, 6, "click:next hold:open");
    oled_text(0, 7, "double-click: back");
}

static void draw_status(void)
{
    char line[24];
    float left, right;

    title(" Status");
    snprintf(line, sizeof line, "Battery %6.2f V", (double)s_vbat);
    oled_text(0, 1, line);
    const uint32_t ms = HAL_GetTick();
    snprintf(line, sizeof line, "Uptime  %6lu.%01lu s",
             (unsigned long)(ms / 1000U), (unsigned long)(ms / 100U % 10U));
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "Head %s", s_boot.heading == HEADING_DRIVE_END_FRONT ? "drive end" : "omni end");
    oled_text(0, 3, line);
    wheel_revs(&left, &right);
    snprintf(line, sizeof line, "Left   %+7.2f rev", (double)left);
    oled_text(0, 4, line);
    snprintf(line, sizeof line, "Right  %+7.2f rev", (double)right);
    oled_text(0, 5, line);
    oled_text(0, 6, "hold: zero wheels");
    oled_text(0, 7, "double-click: back");
}

static void draw_heading(void)
{
    title(" Heading");
    for (int i = 0; i < 2; i++) {
        char line[24];
        snprintf(line, sizeof line, "%s%s%s", i == s_cursor ? ">" : " ",
                 (heading_t)i == s_saved.heading ? "*" : " ", k_heading_labels[i]);
        oled_text(0, 2 + i, line);
        if (i == s_cursor) {
            oled_invert_row(2 + i);
        }
    }
    oled_text(0, 5, s_message);
    oled_text(0, 6, "hold: save  *=saved");
    oled_text(0, 7, "double-click: back");
}

static void draw_about(void)
{
    char line[24];

    title(" About");
    snprintf(line, sizeof line, "HW   %s", board_version_name(s_id.version));
    oled_text(0, 1, line);
    snprintf(line, sizeof line, "IMU  %s", board_imu_name(s_id.imu));
    oled_text(0, 2, line);
    snprintf(line, sizeof line, "UID  %08lX", (unsigned long)HAL_GetUIDw2());
    oled_text(0, 3, line);
    snprintf(line, sizeof line, "  %08lX-%08lX", (unsigned long)HAL_GetUIDw1(), (unsigned long)HAL_GetUIDw0());
    oled_text(0, 4, line);
    oled_text(0, 5, "Build " __DATE__);
    oled_text(0, 7, "double-click: back");
}

static void render(void)
{
    oled_clear();
    switch (s_screen) {
    case SCR_MAIN:    draw_main(); break;
    case SCR_STATUS:  draw_status(); break;
    case SCR_HEADING: draw_heading(); break;
    case SCR_ABOUT:   draw_about(); break;
    }
    oled_flush();
}

static void show(screen_t scr)
{
    s_screen = scr;
    s_cursor = 0;
    if (scr == SCR_HEADING) {
        s_cursor = (int)s_saved.heading;
        s_message = s_saved.heading == s_boot.heading ? "" : "reboot to apply";
    }
}

static void save_heading(heading_t h)
{
    if (h == s_saved.heading) {
        s_message = "already saved";
        return;
    }
    /* 抹除時 CPU 停住約 1.2 s，先把提示畫出來。 */
    s_message = "saving...";
    render();
    settings_t next = s_saved;
    next.heading = h;
    if (settings_save(&next)) {
        s_saved = next;
        s_message = h == s_boot.heading ? "saved" : "saved, reboot needed";
        printf("heading saved: %s (%s)\n", heading_name(h),
               h == s_boot.heading ? "same as this boot" : "applies after reboot");
    } else {
        s_message = "SAVE FAILED";
        printf("heading save FAILED\n");
    }
}

static void handle(key_event_t e)
{
    switch (s_screen) {
    case SCR_MAIN:
        if (e == KEY_CLICK) {
            s_cursor = (s_cursor + 1) % MAIN_COUNT;
        } else if (e == KEY_LONG) {
            show((screen_t)(SCR_STATUS + s_cursor));
        }
        break;
    case SCR_STATUS:
        if (e == KEY_LONG) {
            for (int i = 0; i < ENC_COUNT; i++) {
                s_zero[i] = s_counts[i];
            }
        } else if (e == KEY_DOUBLE) {
            show(SCR_MAIN);
            s_cursor = 0;
        }
        break;
    case SCR_HEADING:
        if (e == KEY_CLICK) {
            s_cursor = (s_cursor + 1) % 2;
            s_message = "";
        } else if (e == KEY_LONG) {
            save_heading((heading_t)s_cursor);
        } else if (e == KEY_DOUBLE) {
            show(SCR_MAIN);
            s_cursor = 1;
        }
        break;
    case SCR_ABOUT:
        if (e == KEY_DOUBLE) {
            show(SCR_MAIN);
            s_cursor = 2;
        }
        break;
    }
}

static void measure_battery(void)
{
    battery_reading_t b;
    s_vbat = battery_measure(&b) ? b.vbat : 0.0f;
}

int main(void)
{
    board_init();
    printf("\n==== oled_menu ====\n");

    s_id = board_id_detect();
    if (s_id.version != HW_V1_0 && s_id.version != HW_V1_1) {
        printf("hardware %s: board.h OLED pins are for V1.0/V1.1, stopping\n",
               board_version_name(s_id.version));
        for (;;) {
        }
    }
    const bool loaded = settings_load(&s_boot);
    s_saved = s_boot;
    printf("heading: %s (%s)\n", heading_name(s_boot.heading),
           loaded ? "loaded from flash" : "no saved settings, default");

    oled_init();
    battery_init();
    encoder_init();
    measure_battery();
    render();

    uint32_t last_status = HAL_GetTick();
    unsigned status_updates = 0;
    for (;;) {
        encoder_poll(s_counts);
        const key_event_t e = key_poll();
        if (e != KEY_NONE) {
            printf("key: %s\n", key_event_name(e));
            handle(e);
            render();
        }
        if (s_screen == SCR_STATUS && HAL_GetTick() - last_status >= STATUS_PERIOD_MS) {
            last_status = HAL_GetTick();
            if (++status_updates % BATTERY_EVERY == 0U) {
                measure_battery();
            }
            render();
        }
        HAL_Delay(LOOP_MS);
    }
}
