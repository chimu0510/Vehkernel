/*
 * 存在內部 Flash 的控制板設定（目前只有車頭方向，見 ADR-0001）。
 *
 * 使用最後一個 sector（sector 7，0x08060000），一筆紀錄含 magic、版本與 CRC-32；
 * 燒錄範例不會抹除這個 sector，但 13_flash_params 會把它清掉並寫入自己的紀錄。
 * 車頭方向只在開機時載入，存檔後重開機才生效，不支援運行中切換。
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>

typedef enum {
    HEADING_DRIVE_END_FRONT = 0,   /* 驅動端在前，原廠預設 */
    HEADING_OMNI_END_FRONT = 1,    /* 全向輪端在前 */
} heading_t;

typedef struct {
    heading_t heading;
} settings_t;

/* 沒有有效紀錄時填入預設值（驅動端在前）並回傳 false。 */
bool settings_load(settings_t *out);

/*
 * 抹除 sector 後寫入並讀回驗證。抹除期間 CPU 停住約 1.2 s（連中斷都不會執行），
 * 只能在底盤靜止、馬達關閉時呼叫。
 */
bool settings_save(const settings_t *s);

const char *heading_name(heading_t h);

#endif /* SETTINGS_H */
