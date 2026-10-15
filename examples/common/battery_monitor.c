#include "battery_monitor.h"

#include "battery.h"
#include "board.h"

static batt_status_t s_status;
static float s_window[BATT_FILTER_LEN];
static unsigned s_window_pos;
static float s_sim_v;                /* > 0 時使用模擬電壓 */
static uint32_t s_last_sample;
static batt_level_t s_pending;       /* 等待持續 BATT_HOLD_MS 的新等級 */
static uint32_t s_pending_since;

/* 依目前等級套用遲滯，回傳電壓 v 對應的等級。 */
static batt_level_t classify(float v, batt_level_t current)
{
    if (v < BATT_PRESENT_MIN_V) {
        return BATT_NONE;
    }
    if (v < BATT_CRITICAL_ON_V || (current == BATT_CRITICAL && v < BATT_CRITICAL_OFF_V)) {
        return BATT_CRITICAL;
    }
    const bool low = current == BATT_LOW || current == BATT_CRITICAL;
    return v < (low ? BATT_LOW_OFF_V : BATT_LOW_ON_V) ? BATT_LOW : BATT_OK;
}

static int percent(float v)
{
    const float p = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
    return p <= 0.0f ? 0 : p >= 100.0f ? 100 : (int)(p + 0.5f);
}

static float sample(void)
{
    battery_reading_t r;

    s_status.simulated = s_sim_v > 0.0f;
    if (s_status.simulated) {
        return s_sim_v;
    }
    return battery_measure(&r) ? r.vbat : 0.0f;
}

static void fill_window(float v)
{
    for (unsigned i = 0; i < BATT_FILTER_LEN; i++) {
        s_window[i] = v;
    }
}

void battery_monitor_init(void)
{
    battery_init();
    s_last_sample = HAL_GetTick();
    s_status.vbat = sample();
    fill_window(s_status.vbat);
    s_status.vbat_filtered = s_status.vbat;
    /* 開機時沒有「目前等級」，以進入門檻直接判斷。 */
    s_status.level = classify(s_status.vbat, BATT_OK);
    s_pending = s_status.level;
    s_status.percent = percent(s_status.vbat);
}

bool battery_monitor_update(void)
{
    const uint32_t now = HAL_GetTick();
    if (now - s_last_sample < BATT_SAMPLE_MS) {
        return false;
    }
    s_last_sample = now;

    s_status.vbat = sample();
    s_window[s_window_pos] = s_status.vbat;
    s_window_pos = (s_window_pos + 1U) % BATT_FILTER_LEN;
    float sum = 0.0f;
    for (unsigned i = 0; i < BATT_FILTER_LEN; i++) {
        sum += s_window[i];
    }
    s_status.vbat_filtered = sum / (float)BATT_FILTER_LEN;
    s_status.percent = percent(s_status.vbat_filtered);

    /* 拔掉或接上電池時平均值要 1 s 才跟上，所以有無電池直接看這一筆。 */
    const batt_level_t target = s_status.vbat < BATT_PRESENT_MIN_V
        ? BATT_NONE
        : classify(s_status.vbat_filtered, s_status.level == BATT_NONE ? BATT_OK : s_status.level);
    if (target == s_status.level) {
        s_pending = target;
        return false;
    }
    if (target != s_pending) {
        s_pending = target;
        s_pending_since = now;
    }
    const bool immediate = target == BATT_NONE || s_status.level == BATT_NONE;
    if (!immediate && now - s_pending_since < BATT_HOLD_MS) {
        return false;
    }
    if (s_status.level == BATT_NONE) {
        /* 剛接上電池：平均值還含有 USB 供電時的低讀值，重新填滿。 */
        fill_window(s_status.vbat);
        s_status.vbat_filtered = s_status.vbat;
        s_status.percent = percent(s_status.vbat);
        s_pending = classify(s_status.vbat, BATT_OK);
    }
    s_status.level = s_pending;
    return true;
}

const batt_status_t *battery_monitor_status(void)
{
    return &s_status;
}

void battery_monitor_simulate(float v)
{
    s_sim_v = v > 0.0f ? v : 0.0f;
}

const char *batt_level_name(batt_level_t level)
{
    switch (level) {
    case BATT_OK:       return "OK";
    case BATT_LOW:      return "LOW";
    case BATT_CRITICAL: return "CRITICAL";
    default:            return "NONE";
    }
}
