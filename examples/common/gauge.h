/*
 * OLED 儀表：畫進 oled 的畫面緩衝，呼叫者負責 oled_flush()。
 * 角度單位為度；航向角 0 朝上、逆時針為正。
 */
#ifndef GAUGE_H
#define GAUGE_H

/*
 * 航向儀：外圈固定，指針指向航向角。
 *   外圈每 30 度一短刻度、0/90/180/270 度為長刻度，頂端外側的三角形標出 0 度；
 *   內圈弧線從 0 度沿轉動方向畫到目前航向；指針為實心箭頭加反向短尾。
 * 需要的空間：以 (cx, cy) 為中心、半徑 r + 4 的方形。
 */
void gauge_heading(int cx, int cy, int r, float yaw_deg);

/*
 * 人工地平儀（ROS REP-103：roll 正 = 右側下沉、pitch 正 = 機頭朝下）。
 *   圓形視窗內上方為天空（空白）、下方為地面（網點），地平線依 roll 旋轉、依 pitch 平移
 *   （每度 px_per_deg 像素）；頂端固定 0/±30/±60 度刻度，內側三角形指出目前 roll；
 *   中央固定的機翼符號代表載具。
 */
void gauge_horizon(int cx, int cy, int r, float roll_deg, float pitch_deg, float px_per_deg);

#endif /* GAUGE_H */
