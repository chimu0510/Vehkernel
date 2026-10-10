/*
 * 使用者按鍵（PC5）的事件：去彈跳後分辨單擊、雙擊與長按，手勢與原廠代碼相同。
 *   - 長按：按住 KEY_LONG_MS，按住期間就送出，放開前不再有事件。
 *   - 雙擊：放開後 KEY_DOUBLE_GAP_MS 內再按下，按下當下送出。
 *   - 單擊：放開後 KEY_DOUBLE_GAP_MS 內沒有再按，才送出；所以單擊會延遲這段時間。
 */
#ifndef KEY_H
#define KEY_H

#define KEY_DEBOUNCE_MS    20U
#define KEY_LONG_MS        600U
#define KEY_DOUBLE_GAP_MS  300U

typedef enum {
    KEY_NONE = 0,
    KEY_CLICK,
    KEY_DOUBLE,
    KEY_LONG,
} key_event_t;

/* 以 HAL_GetTick() 計時；至少每 10 ms 呼叫一次，每次最多回傳一個事件。 */
key_event_t key_poll(void);

const char *key_event_name(key_event_t e);

#endif /* KEY_H */
