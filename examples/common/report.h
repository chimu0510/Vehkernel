/*
 * 範例的測試結果回報。每個檢查輸出一行：
 *   [PASS] <名稱>: <細節>
 *   [FAIL] <名稱>: <細節>
 *   [SKIP] <名稱>: <原因>
 * 最後一行固定為 "RESULT: PASS (n/m)" 或 "RESULT: FAIL (n/m)"，方便工具擷取；
 * 同時 LED 綠燈表示全部通過、紅燈表示有失敗。
 */
#ifndef REPORT_H
#define REPORT_H

#include <stdbool.h>

void report_begin(const char *example);

/* 回傳 ok，方便寫成 if (!report_check(...)) 提早結束。 */
bool report_check(bool ok, const char *name, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

void report_skip(const char *name, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* 印出總結並點亮結果 LED；回傳是否全部通過（SKIP 不算失敗）。 */
bool report_end(void);

/* 印出結束提示並停住；給測完就結束、不再持續輸出的範例使用。 */
void report_halt(void) __attribute__((noreturn));

#endif /* REPORT_H */
