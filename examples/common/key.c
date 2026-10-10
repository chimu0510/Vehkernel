#include "key.h"

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

typedef enum {
    ST_IDLE,
    ST_DOWN,           /* 第一次按下，等長按或放開 */
    ST_UP,             /* 第一次放開，等第二次按下或逾時成單擊 */
    ST_WAIT_RELEASE,   /* 已送出長按或雙擊，等放開 */
} state_t;

static state_t s_state = ST_IDLE;
static uint32_t s_since;          /* 進入目前狀態的時間 */
static bool s_stable;             /* 去彈跳後的按下狀態 */
static bool s_last_raw;
static uint32_t s_raw_since;      /* 原始電位最近一次改變的時間 */

/* 原始電位維持 KEY_DEBOUNCE_MS 不變才更新穩定狀態；回傳穩定狀態是否改變。 */
static bool debounce(uint32_t now)
{
    const bool raw = user_key_pressed();
    if (raw != s_last_raw) {
        s_last_raw = raw;
        s_raw_since = now;
    }
    if (raw != s_stable && now - s_raw_since >= KEY_DEBOUNCE_MS) {
        s_stable = raw;
        return true;
    }
    return false;
}

static void enter(state_t st, uint32_t now)
{
    s_state = st;
    s_since = now;
}

key_event_t key_poll(void)
{
    const uint32_t now = HAL_GetTick();
    const bool changed = debounce(now);
    const bool pressed = changed && s_stable;
    const bool released = changed && !s_stable;

    switch (s_state) {
    case ST_IDLE:
        if (pressed) {
            enter(ST_DOWN, now);
        }
        break;
    case ST_DOWN:
        if (released) {
            enter(ST_UP, now);
        } else if (now - s_since >= KEY_LONG_MS) {
            enter(ST_WAIT_RELEASE, now);
            return KEY_LONG;
        }
        break;
    case ST_UP:
        if (pressed) {
            enter(ST_WAIT_RELEASE, now);
            return KEY_DOUBLE;
        }
        if (now - s_since >= KEY_DOUBLE_GAP_MS) {
            enter(ST_IDLE, now);
            return KEY_CLICK;
        }
        break;
    case ST_WAIT_RELEASE:
        if (released) {
            enter(ST_IDLE, now);
        }
        break;
    }
    return KEY_NONE;
}

const char *key_event_name(key_event_t e)
{
    switch (e) {
    case KEY_CLICK:  return "click";
    case KEY_DOUBLE: return "double";
    case KEY_LONG:   return "long";
    default:         return "none";
    }
}
