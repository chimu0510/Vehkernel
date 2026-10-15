#include "oled.h"

#include <string.h>

#include "board.h"
#include "font5x7.h"

/*
 * SSD1306 的 SPI 時脈週期最短 100 ns；這裡每半週期等 34 個 CPU 週期（168 MHz 下約 200 ns），
 * 約 2.5 MHz，留足餘量給杜邦線與排針。
 */
#define HALF_PERIOD_CYCLES 34U

/* 頁面緩衝：8 頁 × 128 欄，每個位元組是同一欄由上而下的 8 個像素（bit0 在上）。 */
static uint8_t s_fb[OLED_HEIGHT / 8][OLED_WIDTH];

static void pin(GPIO_TypeDef *port, uint16_t p, bool high)
{
    HAL_GPIO_WritePin(port, p, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void half_period(void)
{
    const uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < HALF_PERIOD_CYCLES) {
    }
}

/* MSB 先送，SSD1306 在 SCL 上升緣取樣。 */
static void write_byte(uint8_t byte, bool data)
{
    pin(OLED_DC_PORT, OLED_DC_PIN, data);
    for (int i = 7; i >= 0; i--) {
        pin(OLED_SCL_PORT, OLED_SCL_PIN, false);
        pin(OLED_SDA_PORT, OLED_SDA_PIN, (byte >> i) & 1U);
        half_period();
        pin(OLED_SCL_PORT, OLED_SCL_PIN, true);
        half_period();
    }
}

static void cmd(uint8_t c)
{
    write_byte(c, false);
}

void oled_init(void)
{
    GPIO_InitTypeDef io = {0};

    io.Mode = GPIO_MODE_OUTPUT_PP;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_MEDIUM;
    io.Pin = OLED_SCL_PIN;
    HAL_GPIO_Init(OLED_SCL_PORT, &io);
    io.Pin = OLED_SDA_PIN;
    HAL_GPIO_Init(OLED_SDA_PORT, &io);
    io.Pin = OLED_RST_PIN;
    HAL_GPIO_Init(OLED_RST_PORT, &io);
    io.Pin = OLED_DC_PIN;
    HAL_GPIO_Init(OLED_DC_PORT, &io);

    pin(OLED_SCL_PORT, OLED_SCL_PIN, true);
    pin(OLED_RST_PORT, OLED_RST_PIN, false);
    HAL_Delay(10);                  /* RES# 低電位至少 3 us */
    pin(OLED_RST_PORT, OLED_RST_PIN, true);
    HAL_Delay(10);

    static const uint8_t k_init[] = {
        0xAE,         /* 關閉顯示 */
        0xD5, 0x80,   /* 時脈分頻 1、振盪頻率預設 */
        0xA8, 0x3F,   /* 多工比 1/64 */
        0xD3, 0x00,   /* 顯示偏移 0 */
        0x40,         /* 起始行 0 */
        0x8D, 0x14,   /* 開啟內建電荷泵（模組只接 3.3 V） */
        0x20, 0x02,   /* 頁定址模式 */
        0xA1,         /* 欄 127 對應 SEG0 */
        0xC8,         /* COM 由下往上掃描；與 0xA1 合起來是原廠代碼的顯示方向 */
        0xDA, 0x12,   /* COM 腳位：交錯，128×64 用 */
        0x81, 0xEF,   /* 對比度，與原廠代碼相同 */
        0xD9, 0xF1,   /* 預充電週期 */
        0xDB, 0x30,   /* VCOMH 約 0.83 VCC */
        0xA4,         /* 依 RAM 顯示 */
        0xA6,         /* 正常（不反相） */
    };
    for (unsigned i = 0; i < sizeof k_init; i++) {
        cmd(k_init[i]);
    }
    oled_clear();
    oled_flush();
    cmd(0xAF);                      /* 開啟顯示 */
    HAL_Delay(100);                 /* 電荷泵穩定 */
}

void oled_clear(void)
{
    memset(s_fb, 0, sizeof s_fb);
}

void oled_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
        return;
    }
    const uint8_t mask = (uint8_t)(1U << (y % 8));
    if (on) {
        s_fb[y / 8][x] |= mask;
    } else {
        s_fb[y / 8][x] &= (uint8_t)~mask;
    }
}

void oled_rect(int x, int y, int w, int h, bool on)
{
    for (int i = 0; i < w; i++) {
        oled_pixel(x + i, y, on);
        oled_pixel(x + i, y + h - 1, on);
    }
    for (int j = 0; j < h; j++) {
        oled_pixel(x, y + j, on);
        oled_pixel(x + w - 1, y + j, on);
    }
}

void oled_text(int col, int row, const char *s)
{
    if (row < 0 || row >= OLED_HEIGHT / OLED_CHAR_H) {
        return;
    }
    for (int x = col * OLED_CHAR_W; *s != '\0' && x + OLED_CHAR_W <= OLED_WIDTH; s++) {
        const char c = (*s >= FONT5X7_FIRST && *s <= FONT5X7_LAST) ? *s : '?';
        const uint8_t *glyph = k_font5x7[c - FONT5X7_FIRST];
        for (int i = 0; i < 5; i++) {
            s_fb[row][x++] = glyph[i];
        }
        s_fb[row][x++] = 0;
    }
}

void oled_invert_row(int row)
{
    if (row < 0 || row >= OLED_HEIGHT / OLED_CHAR_H) {
        return;
    }
    for (int x = 0; x < OLED_WIDTH; x++) {
        s_fb[row][x] ^= 0xFFU;
    }
}

/* Bresenham 直線。 */
void oled_line(int x0, int y0, int x1, int y1, bool on)
{
    const int dx = x1 > x0 ? x1 - x0 : x0 - x1, dy = -(y1 > y0 ? y1 - y0 : y0 - y1);
    const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        oled_pixel(x0, y0, on);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

/* 中點畫圓法。 */
void oled_circle(int cx, int cy, int r, bool on)
{
    int x = r, y = 0, err = 1 - r;

    while (x >= y) {
        const int px[8] = {x, y, -y, -x, -x, -y, y, x};
        const int py[8] = {y, x, x, y, -y, -x, -x, -y};
        for (int i = 0; i < 8; i++) {
            oled_pixel(cx + px[i], cy + py[i], on);
        }
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

void oled_disc(int cx, int cy, int r, bool on)
{
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r * r) {
                oled_pixel(cx + dx, cy + dy, on);
            }
        }
    }
}

/* 以邊函數判斷包圍盒內每個像素是否在三角形內（含邊上），與頂點順序無關。 */
void oled_triangle(int x0, int y0, int x1, int y1, int x2, int y2, bool on)
{
    const int minx = x0 < x1 ? (x0 < x2 ? x0 : x2) : (x1 < x2 ? x1 : x2);
    const int maxx = x0 > x1 ? (x0 > x2 ? x0 : x2) : (x1 > x2 ? x1 : x2);
    const int miny = y0 < y1 ? (y0 < y2 ? y0 : y2) : (y1 < y2 ? y1 : y2);
    const int maxy = y0 > y1 ? (y0 > y2 ? y0 : y2) : (y1 > y2 ? y1 : y2);

    for (int y = miny; y <= maxy; y++) {
        for (int x = minx; x <= maxx; x++) {
            const int e0 = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0);
            const int e1 = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1);
            const int e2 = (x0 - x2) * (y - y2) - (y0 - y2) * (x - x2);
            if ((e0 >= 0 && e1 >= 0 && e2 >= 0) || (e0 <= 0 && e1 <= 0 && e2 <= 0)) {
                oled_pixel(x, y, on);
            }
        }
    }
    oled_line(x0, y0, x1, y1, on);
    oled_line(x1, y1, x2, y2, on);
    oled_line(x2, y2, x0, y0, on);
}

void oled_flush(void)
{
    for (unsigned page = 0; page < OLED_HEIGHT / 8; page++) {
        cmd((uint8_t)(0xB0 + page));
        cmd(0x00);                  /* 欄位址低 4 位元 */
        cmd(0x10);                  /* 欄位址高 4 位元 */
        for (unsigned x = 0; x < OLED_WIDTH; x++) {
            write_byte(s_fb[page][x], true);
        }
    }
}

void oled_all_on(bool on)
{
    cmd(on ? 0xA5 : 0xA4);
}

void oled_invert(bool on)
{
    cmd(on ? 0xA7 : 0xA6);
}

void oled_display_on(bool on)
{
    cmd(on ? 0xAF : 0xAE);
}
