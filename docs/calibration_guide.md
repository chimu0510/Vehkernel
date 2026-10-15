# ROS2-UGV 物理參數與感測器標定指南

本文件說明 WHEELTEC C50C 底盤之物理動態參數標定程序（有效輪徑、有效輪距、減速度）及感測器外參（LiDAR / 相機）與視野（FOV）遮擋處理規範。

依據決策：
- [ADR-0001: 軟體定義車頭方向](file:///home/gugugaga/WorkSpace/MCU/ROS2-UGV/docs/adr/0001-heading-defined-in-software.md)
- [ADR-0002: 運動學解算與角速度閉環下沉下位機](file:///home/gugugaga/WorkSpace/MCU/ROS2-UGV/docs/adr/0002-kinematics-and-yaw-loop-on-mcu.md)
- [ADR-0004: 控制來源仲裁與失效保護狀態機](file:///home/gugugaga/WorkSpace/MCU/ROS2-UGV/docs/adr/0004-control-source-arbitration-and-failsafe.md)

---

## 1. 底盤運動學參數標定

底盤使用充氣橡膠胎與四輪差速結構（兩驅動輪 + 兩被動全向輪）。實際行駛中，胎壓、負載、地面摩擦與全向輪側向拖拽均會造成輪胎有效半徑及轉向有效輪距與幾何設計值產生偏差。

標定工具使用：[`tools/calibrate_chassis.py`](file:///home/gugugaga/WorkSpace/MCU/ROS2-UGV/tools/calibrate_chassis.py)

### 1.1 有效輪徑標定 (Linear Scale)
* **目的**：修正編碼器計數換算為前進線位移的比例。
* **名義參數**：輪徑 $D_{nominal} = 0.208\text{ m}$（周長 $\approx 0.65345\text{ m}$，單圈編碼器 36000 counts）。
* **操作步驟**：
  1. 將車體放置於平整硬質地面，以捲尺標記起點線。
  2. 執行標定命令：
     ```bash
     python3 tools/calibrate_chassis.py --port /dev/ttyACM0 --heading drive_front --mode linear --dist 2.0
     ```
  3. 車體以 $0.25\text{ m/s}$ 勻速前進預設距離後平滑煞停。
  4. 測量驅動輪接觸地面中心點之實際位移距離 $D_{real}$。
  5. 終端機輸入測量值，程式自動計算修正係數：
     $$\text{linear\_scale} = \frac{D_{real}}{D_{measured}}$$
     $$D_{calibrated} = D_{nominal} \times \text{linear\_scale}$$
  6. 將 `linear_scale` 寫入 `ugv_base` 啟動參數或更新 C50C 韌體參數。

### 1.2 有效輪距標定 (Angular Scale)
* **目的**：修正差速轉向時兩輪差速計算轉向角速度 $\omega_z$ 的等效輪距。由於差速旋轉存在輪胎滑移與全向輪摩擦阻力，有效輪距 $W_{eff}$ 通常大於幾何輪距 $W_{geom}$。
* **名義參數**：輪距 $W_{nominal} = 0.350\text{ m}$。
* **操作步驟**：
  1. 在車身與地面標記一致的零度方位基準線。
  2. 執行旋轉標定命令（連續自轉 5 圈）：
     ```bash
     python3 tools/calibrate_chassis.py --port /dev/ttyACM0 --heading drive_front --mode angular --turns 5
     ```
  3. 車體完成預定旋轉圈數後停止。
  4. 觀察車體方位線與地面基準線的實際角度誤差（以角度計）。
  5. 輸入實際旋轉總角度（例如 5 圈整則輸入 1800.0），計算輪距修正係數：
     $$\text{angular\_scale} = \frac{\theta_{real}}{\theta_{measured}}$$
     $$W_{calibrated} = W_{nominal} \times \text{angular\_scale}$$

### 1.3 最大減速度與斜坡煞車測試 (Failsafe Deceleration)
* **目的**：驗證 [ADR-0004](file:///home/gugugaga/WorkSpace/MCU/ROS2-UGV/docs/adr/0004-control-source-arbitration-and-failsafe.md) 定義的平滑減速斜坡，確保在最大行駛速度下觸發失效保護或急停時，車體不發生打滑失控、側翻或齒輪衝擊損壞。
* **操作步驟**：
  1. 確保前方具備 $3\text{ m}$ 以上開闊緩衝區。
  2. 執行測試：
     ```bash
     python3 tools/calibrate_chassis.py --port /dev/ttyACM0 --mode failsafe --speed 0.8
     ```
  3. 車體加速至 $0.8\text{ m/s}$ 後模擬斷開通訊心跳，觸發下位機失效保護。
  4. 觀測遙測回報之減速斜坡曲線與停止時間 $t_{stop}$：
     $$a_{brake} \approx \frac{V_{max}}{t_{stop}} \le a_{max\_safe} \approx 1.5\text{ m/s}^2$$

---

## 2. 感測器外參標定 (Sensor Extrinsics)

依據 [ADR-0001](file:///home/gugugaga/WorkSpace/MCU/ROS2-UGV/docs/adr/0001-heading-defined-in-software.md)，`base_link` 定位在驅動軸中點，且感測器僅安裝於被選定為車頭方向那一端的「感測器支架」上。

```
[驅動端]                                         [全向輪端]
 +-------+                                       +-------+
 | 驅動輪 |=====[ base_link (驅動軸心) ]==========| 全向輪 |
 +-------+         |                             +-------+
                   | (wheelbase = 0.370m)
 支架 A            v                              支架 B
(驅動端感測器支架)                                (全向輪端感測器支架)
```

### 2.1 坐標轉換矩陣關係
1. **當 `heading = drive_front`（驅動端在前）時**：
   * 車身朝向即為 `base_link` 的 $+X$ 方向。
   * 感測器安裝於驅動端支架（`sensor_mount_drive_end`）：
     * $X = +0.080\text{ m}$
     * $Y = 0.000\text{ m}$
     * $Z = +0.160\text{ m}$
     * $\text{RPY} = (0, 0, 0)$
   * LiDAR 坐標系 `laser_link`：
     * 相對於 `sensor_mount_active`：$X = 0.0, Y = 0.0, Z = +0.050\text{ m}$，朝向正前。

2. **當 `heading = omni_front`（全向輪端在前）時**：
   * 車身物理本體旋轉 $180^\circ$（$\pi\text{ rad}$），全向輪端朝向 `base_link` 的 $+X$ 方向。
   * 感測器安裝於全向輪端支架（`sensor_mount_omni_end`）：
     * 物理位置相對於底盤中點在全向輪側，但在 `base_link` 下自動映射為車頭前向。
     * URDF 宏將 `sensor_mount_active` 自動切換至前向掛載點，保持光達與相機的 X 軸正向永遠朝向當前車頭前進方向。

### 2.2 外參實測校正方法
1. **LiDAR 外參標定**：
   * 將車體精準正對平整牆面（距離 $1.000\text{ m}$）。
   * 啟動 `ugv_description` 與雷達節點，在 RViz2 中觀察牆面點雲：
     * 若點雲中心距為 $0.920\text{ m}$，且實際量測雷達到牆為 $0.920\text{ m}$，則 $X$ 軸外參無誤。
     * 若牆面點雲呈現傾斜（非平行於 $Y$ 軸），則微調 `laser_joint` 之偏航角 $\text{yaw}$ 直至點雲垂直於 $X$ 軸。
2. **IMU 零偏與對齊**：
   * ICM-20948 安裝於 C50C 核心板中心。
   * 上位機靜止狀態下，檢查 `/imu/data_raw` 的角速度 $\omega_z$ 漂移（應 $< 0.01\text{ rad/s}$）。

---

## 3. 感測器視野（FOV）遮擋與濾波處理

當雷達架設在車頂支架時，視線可能被底盤天線、提把或車邊輪廓邊緣部分遮擋：

1. **遮擋區域分析**：
   * 360° 2D LiDAR 若安裝在前端支架，其正後方（$\approx 135^\circ \sim 225^\circ$ 或 $-180^\circ \pm 45^\circ$）視角會掃描到車身後部結構。
   * 這些近距離噪點（距離 $< 0.35\text{ m}$）若進入 Costmap，會導致車體將自身邊緣視為障礙物而無法移動。

2. **防護配置方案**：
   * **方法 A：Costmap 最小有效距離過濾**
     在 `nav2_params_*.yaml` 中設定 `raytrace_min_range: 0.38` 與 `obstacle_min_range: 0.38`，自動忽略車身內部的雷達回波。
   * **方法 B：LaserScan 角度過濾 (`laser_filters`)**
     使用 `laser_filters/LaserScanAngularBoundsFilter` 裁切遮擋扇區：
     ```yaml
     scan_filter_chain:
     - name: chassis_shadow_filter
       type: laser_filters/LaserScanAngularBoundsFilter
       params:
         lower_angle: -2.35  # -135 deg
         upper_angle: 2.35   # +135 deg
     ```
   * **方法 C：Footprint 多邊形嚴格對齊**
     確保護邊輪廓完整包覆底盤及輪胎幾何邊界，Costmap 的 clearing ray 會自動清除 Footprint 範圍內的虛假障礙物。
