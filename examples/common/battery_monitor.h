/*
 * 電池監控：定時量測電池電壓、濾波，並判斷電量等級（見 CONTEXT.md「低電量」）。
 *
 * 門檻沿用原廠代碼（24 V 系統，BALANCE/show_task.c、balance_task.c）：
 *   低於 20.0 V 為低電量（原廠此時關閉馬達），回升到 21.0 V 才解除；
 *   電量百分比以 20.0 V = 0%、23.05 V = 100% 線性換算，只是粗略指示。
 * 本模組另外加上：
 *   - 嚴重低電量：低於 18.0 V，回升到 19.0 V 才回到低電量。
 *   - 無電池：低於 6 V，通常是只接 USB 供電；立即切換。
 *   - 其他等級切換需要新等級持續 BATT_HOLD_MS，避免馬達啟動時的瞬間壓降誤判。
 */
#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

#define BATT_LOW_ON_V        20.0f
#define BATT_LOW_OFF_V       21.0f
#define BATT_CRITICAL_ON_V   18.0f
#define BATT_CRITICAL_OFF_V  19.0f
#define BATT_PRESENT_MIN_V   6.0f
#define BATT_EMPTY_V         20.0f    /* 0% */
#define BATT_FULL_V          23.05f   /* 100% */

#define BATT_SAMPLE_MS       100U
#define BATT_FILTER_LEN      10U      /* 移動平均 1 s */
#define BATT_HOLD_MS         2000U

typedef enum {
    BATT_NONE = 0,     /* 沒有電池（只有 USB 供電） */
    BATT_OK,
    BATT_LOW,
    BATT_CRITICAL,
} batt_level_t;

typedef struct {
    float vbat;           /* 最近一次量測，V */
    float vbat_filtered;  /* 移動平均，V；等級判斷用這個 */
    batt_level_t level;
    int percent;          /* 0–100 */
    bool simulated;       /* 目前用的是模擬電壓 */
} batt_status_t;

/* 需要先呼叫 board_init()；會初始化 ADC 並量第一筆。 */
void battery_monitor_init(void);

/* 至少每 BATT_SAMPLE_MS 呼叫一次；回傳等級是否在這次呼叫中改變。 */
bool battery_monitor_update(void);

const batt_status_t *battery_monitor_status(void);

/* 以模擬電壓取代 ADC 量測，用來驗證門檻；v <= 0 取消模擬。 */
void battery_monitor_simulate(float v);

const char *batt_level_name(batt_level_t level);

#endif /* BATTERY_MONITOR_H */
