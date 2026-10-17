# 測試分五層，SITL 跑同一套 RTOS

測試分成以下層級：

- **L0 靜態檢查**：`-Werror`、clang-tidy、產生器一致性、板級定義驗證。
- **L1 主機單元測試**：使用 GoogleTest + gmock，與 Betaflight、PX4 相同。
  - 驅動測試時，`platform::` 指向假的匯流排，內含裝置的暫存器模型（ADR 0009）。
  - Link 解碼器用 libFuzzer 做模糊測試。
  - 參數儲存要用假的 Flash，模擬寫入途中任意位置斷電。
- **L2 資料回放**：用錄下的資料重跑估測器，與參考結果比對。資料格式由 blackbox 的 ADR 決定。
- **L3 SITL 情境**：完整韌體在主機上執行，以腳本描述情境（參考 ArduPilot 的 `rover.py`），例如繞正方形檢查里程計誤差、切斷 Link 檢查安全行為、遙控器接管。
- **L4 實機測試**：韌體提供測試模式執行 bring-up 案例，結果透過 Link 回報，由主機端執行器負責燒錄與收集。第一批案例包括：
  - IMU 接線，讀 WHO_AM_I。
  - 版本識別腳。
  - 馬達與編碼器的對應和方向。
  - PWM 頻率。
  - 參數儲存。

  會轉動輪子的測試，須先確認車輪已經離地，並限制輸出功率。
- **L5 ROS2 整合**：SITL + Bridge + Nav2，用 launch_testing 執行。

**SITL**

- SITL 使用 FreeRTOS 官方的 POSIX 移植版，所以 Work Queue、匯流排與排程行為都和實機相同。
- 模擬器分兩種，參考 PX4 的 SIH / `gz_bridge` 與 ArduPilot 的 `SIM_Rover` / `SIM_JSON`：
  - **內建輕量模擬器**：每個 Frame Class 一個物理模型，在同一個行程中執行，CI 預設使用它。
  - **外部模擬器介面**：與模擬器無關的通用協議，訊息由 Schema 定義（ADR 0006），可以接 Gazebo、Webots 或使用者自訂的模擬器。
- SITL 的 Link 透過 pty 或 UDP 接上真正的 Bridge，不需要硬體就能跑完整的 ROS2 堆疊。

**CI**

每次 push 執行：
1. 所有板子的建置與大小預算檢查。
2. L0 與 L1。
3. L2，等格式確定後加入。
4. L3，使用內建模擬器。

L4 之後接上 self-hosted runner，初期先手動執行。
