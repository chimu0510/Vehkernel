#include "gauge.h"

#include <math.h>
#include <stdbool.h>

#include "oled.h"

#define DEG2RAD(d) ((d) * 3.14159265f / 180.0f)

/* 以 (cx, cy) 為圓心、半徑 rho、角度 deg（0 朝上、逆時針為正）的點。 */
static void polar(int cx, int cy, float rho, float deg, int *x, int *y)
{
    const float t = DEG2RAD(deg);
    *x = (int)lroundf((float)cx - rho * sinf(t));
    *y = (int)lroundf((float)cy - rho * cosf(t));
}

static void radial(int cx, int cy, float r0, float r1, float deg)
{
    int x0, y0, x1, y1;
    polar(cx, cy, r0, deg, &x0, &y0);
    polar(cx, cy, r1, deg, &x1, &y1);
    oled_line(x0, y0, x1, y1, true);
}

void gauge_heading(int cx, int cy, int r, float yaw_deg)
{
    const float fr = (float)r;
    int x, y;

    oled_circle(cx, cy, r, true);
    for (int a = 0; a < 360; a += 30) {
        radial(cx, cy, fr - 1.0f, fr - 1.0f - (a % 90 == 0 ? 4.0f : 2.0f), (float)a);
    }
    oled_triangle(cx, cy - r - 2, cx - 2, cy - r - 4, cx + 2, cy - r - 4, true);

    const float sweep = fabsf(yaw_deg), dir = yaw_deg < 0.0f ? -1.0f : 1.0f;
    for (float a = 0.0f; a <= sweep; a += 5.0f) {
        polar(cx, cy, fr - 7.0f, dir * a, &x, &y);
        oled_pixel(x, y, true);
    }

    /* 箭頭：尖端在 r-3，底邊在 r-10、寬 6 px。 */
    const float t = DEG2RAD(yaw_deg);
    const float px = cosf(t), py = -sinf(t);        /* 與指針方向垂直的單位向量 */
    int tx, ty, bx, by;
    polar(cx, cy, fr - 3.0f, yaw_deg, &tx, &ty);
    polar(cx, cy, fr - 10.0f, yaw_deg, &bx, &by);
    oled_triangle(tx, ty,
                  bx - (int)lroundf(3.0f * px), by - (int)lroundf(3.0f * py),
                  bx + (int)lroundf(3.0f * px), by + (int)lroundf(3.0f * py), true);
    oled_line(cx, cy, bx, by, true);
    polar(cx, cy, 6.0f, yaw_deg + 180.0f, &x, &y);
    oled_line(cx, cy, x, y, true);
    oled_disc(cx, cy, 2, true);
}

void gauge_horizon(int cx, int cy, int r, float roll_deg, float pitch_deg, float px_per_deg)
{
    const float s = sinf(DEG2RAD(roll_deg)), c = cosf(DEG2RAD(roll_deg));
    /* pitch 正（機頭朝下）時看到更多地面，地平線往上移；限制在視窗內。 */
    float h = -pitch_deg * px_per_deg;
    h = fmaxf(fminf(h, (float)r), -(float)r);

    /*
     * 把螢幕座標轉回地平線座標：v = x·sin(roll) + y·cos(roll)（y 向下）。
     * v > h 是地面；roll 正時右側地平線升高，看起來就是載具向右傾。
     */
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy > (r - 1) * (r - 1)) {
                continue;
            }
            const float v = (float)dx * s + (float)dy * c;
            if (v > h + 0.5f) {
                oled_pixel(cx + dx, cy + dy, ((dx + dy) & 1) == 0);   /* 網點地面 */
            } else if (v > h - 0.5f) {
                oled_pixel(cx + dx, cy + dy, true);                   /* 地平線 */
            }
        }
    }
    oled_circle(cx, cy, r, true);

    /*
     * 頂端固定刻度與 roll 指示：刻度在外圈內側；三角形跟地平線一起轉，
     * 右傾時地平線逆時針轉、三角形往左偏，與飛機姿態儀相同。
     */
    const float fr = (float)r;
    for (int a = -60; a <= 60; a += 30) {
        radial(cx, cy, fr - 1.0f, fr - (a == 0 ? 5.0f : 3.0f), (float)a);
    }
    int tx, ty, lx, ly, rx, ry;
    polar(cx, cy, fr - 6.0f, roll_deg, &tx, &ty);
    polar(cx, cy, fr - 10.0f, roll_deg + 8.0f, &lx, &ly);
    polar(cx, cy, fr - 10.0f, roll_deg - 8.0f, &rx, &ry);
    oled_triangle(tx, ty, lx, ly, rx, ry, true);

    /* 機翼符號 ─┐ ● ┌─：先在每條線周圍清出 1 px，在網點地面上也看得清楚。 */
    const int w = r / 2;
    const int segs[4][4] = {
        {cx - w, cy, cx - 4, cy}, {cx + 4, cy, cx + w, cy},       /* 左右機翼 */
        {cx - 4, cy, cx - 4, cy + 2}, {cx + 4, cy, cx + 4, cy + 2}, /* 翼尖下折 */
    };
    for (int i = 0; i < 4; i++) {
        for (int oy = -1; oy <= 1; oy++) {
            for (int ox = -1; ox <= 1; ox++) {
                oled_line(segs[i][0] + ox, segs[i][1] + oy, segs[i][2] + ox, segs[i][3] + oy, false);
            }
        }
    }
    oled_disc(cx, cy, 2, false);
    for (int i = 0; i < 4; i++) {
        oled_line(segs[i][0], segs[i][1], segs[i][2], segs[i][3], true);
    }
    oled_disc(cx, cy, 1, true);
}
