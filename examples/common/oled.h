/*
 * SSD1306 128×64 OLED，GPIO 模擬 4 線 SPI（無 CS、只寫不讀）。
 * 繪圖寫進 RAM 中的畫面緩衝，oled_flush() 才送到螢幕。
 * 座標原點在左上角，方向與原廠代碼的顯示方向相同。
 */
#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

/* 字元格 6×8（5×7 字形加 1 欄間距），一行 21 字、共 8 行。 */
#define OLED_CHAR_W 6
#define OLED_CHAR_H 8

void oled_init(void);

void oled_clear(void);
void oled_pixel(int x, int y, bool on);
void oled_rect(int x, int y, int w, int h, bool on);
/* 從字元格 (col, row) 開始寫字串，超出右邊界的部分不畫。 */
void oled_text(int col, int row, const char *s);
/* 反白整個字元行（亮底暗字），用於標題列與選取項目；畫完該行文字後再呼叫。 */
void oled_invert_row(int row);
void oled_flush(void);

/* 直接控制顯示器的命令，不影響畫面緩衝。 */
void oled_all_on(bool on);       /* 忽略 RAM、全部像素點亮 */
void oled_invert(bool on);
void oled_display_on(bool on);

#endif /* OLED_H */
