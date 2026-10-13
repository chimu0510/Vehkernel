# C50C 控制板範例

每個範例是一個獨立的韌體。功能驗證（`bringup/`）的測試結果從 USART1（Type-C 串口 1，115200 8N1）輸出：

```
==== 22_imu ====
[PASS] who_am_i: 0xEA (expect 0xEA)
[FAIL] gravity: |a| = 0.612 g (expect 0.90..1.10)
[SKIP] buzzer: no answer
RESULT: FAIL (8/9 passed, 1 skipped)
```

最後一行固定以 `RESULT:` 開頭，結束時 LED 亮綠燈表示全部通過、亮紅燈表示有失敗。

## 範例列表

範例分兩類：

- `bringup/`：**功能驗證**。每個韌體只測試一個單元，用來確認腳位、極性與板級常數（`common/board.h`）是否正確。
- `demos/`：**範例程序**。在功能驗證通過的單元上做完整功能（例如 OLED 的 UI 菜單），名稱不帶編號，在說明中寫出依賴的單元。

### 功能驗證（`bringup/`）

依依賴層級排序，十位數是層，後面的層假設前面的已經通過。全部範例都用 USART1 主控台輸出結果、用 LED 表示總結，所以 `02_uart` 與 `11_led` 要最先通過。

| 層 | 範例 | 測試內容 | 依賴 | 需要的操作 |
|---|---|---|---|---|
| 0 核心 | `01_board_info` | MCU 型號與 Flash 大小、8 MHz 晶振、168 MHz 時脈、硬體版本、IMU 型號 | 主控台 TX | 無 |
| | `02_uart` | USART1 與 USART3 的收發 | — | 在兩個 Type-C 串口各輸入 `ping` |
| 1 板上基本 I/O | `11_led` | RGB LED 的紅、綠、藍、白、全滅 | 主控台 RX | 看 LED，在主控台回答 y/n |
| | `12_key_buzzer` | 使用者按鍵（PC5）、使能開關（PE4）、蜂鳴器（PD11） | LED | 按一次按鍵、來回切換使能開關、回答是否聽到蜂鳴器；蜂鳴器要插上跳線帽 |
| | `13_flash_params` | 內部 Flash 最後一個 sector 的抹除、寫入、讀回，以及斷電後是否保留 | — | 執行一次後斷電（拔電池與 USB），再執行一次 |
| 2 板上周邊 | `21_oled` | OLED（SSD1306 128×64）全亮、邊框與文字方向、反相、關閉顯示、整個畫面更新時間；之後持續顯示開機時間與按鍵狀態 | 硬體版本（`01`）、按鍵 | 插上 OLED 模組；看螢幕回答 y/n。腳位只適用 V1.0/V1.1 |
| | `22_imu` | ICM-20948 辨識與設定、靜止時的重力與零偏、磁力計、z 軸角速度 | 硬體版本（`01`） | 先靜置，再水平逆時針轉動車子；之後持續輸出量測值 |
| | `23_battery_adc` | VDDA、電池電壓與雜訊；可選與萬用電表比對 | — | 接上電池；可輸入萬用電表讀數 |
| 3 底盤驅動 | `31_encoder` | 四路編碼器靜止穩定；左右驅動輪對應的通道、方向與每圈計數（反推編碼器線數） | 按鍵 | 左、右驅動輪各往驅動端方向用手轉一圈，再按使用者按鍵 |
| | `32_motor` | 馬達 A、B 低速正反轉；帶動哪一路編碼器、反轉時正負號相反、是左輪或右輪 | 按鍵、使能開關、電池電壓、編碼器 | **車子架高、驅動輪離地**；接電池；打開使能開關；回答 y/n。按按鍵或關使能開關立即停止 |
| 4 通訊 | `41_sbus` | SBUS 訊框率與錯誤率、搖桿通道 1–4 的範圍、遙控器關閉時接收機的失效保護行為 | — | 接收機接 CN7 並對頻；推動兩支搖桿到四個角落；依提示關閉再打開遙控器 |
| | `42_can` | CAN1 靜默迴環與迴環；可選與外部 CAN 節點收發 | — | 可選：接 500 kbit/s 的 USB-CAN 轉接器並送出一個訊框 |

「依賴」只列主控台與 LED 以外的單元。尚未有功能驗證的單元：USB 手柄（USB Host，PA11/PA12，屬第 4 層）。航模 PWM 輸入、舵機、藍牙串口、數碼管、選車型電位器與自動回充在本專案用不到，不另寫範例。

### 範例程序（`demos/`）

| 範例 | 功能 | 依賴 | 操作 |
|---|---|---|---|
| `battery_monitor` | 電池電壓濾波與電量等級（OK／低電量／嚴重低電量／無電池，門檻沿用原廠 24 V 系統），以 LED、蜂鳴器、OLED 警告 | `11_led`、`12_key_buzzer`、`21_oled`、`23_battery_adc` | 接上電池；主控台 `sim <V>` 以模擬電壓驗證門檻、`sim off` 取消；長按按鍵靜音 |
| `imu_yaw` | 陀螺儀零偏校正（靜置 2 s，晃動就重來）、200 Hz 控制中斷積分航向角，OLED 顯示航向角指針、角速度、零偏、漂移與中斷執行時間 | `12_key_buzzer`、`21_oled`、`22_imu` | 開機靜置；逆時針轉 90° 應約 +90；長按歸零、雙擊重新校正 |
| `imu_ahrs` | `imu_yaw` 的完整版：Mahony 濾波融合陀螺儀與加速度計估測 roll／pitch／yaw；開機校正三軸零偏與水平基準，靜止時持續更新零偏並鎖住角度；OLED 三頁（人工地平儀、航向儀、感測器狀態） | `12_key_buzzer`、`21_oled`、`22_imu` | 開機靜置；單擊換頁、長按 yaw 歸零、雙擊重新校正 |
| `imu_ekf` | 乘性誤差狀態 EKF：估測姿態與三軸零偏（z 軸靠靜止時的零角速度量測），加速度權重隨 \|a\| 偏離 1 g 自動降低，輸出各角度與零偏的 1σ；同時執行 Mahony 比較 yaw、漂移與零偏 | `12_key_buzzer`、`21_oled`、`22_imu` | 不需開機靜置（Mahony 那一路需要）；單擊換頁、長按 yaw 歸零、雙擊兩者重來 |
| `ahrs_bench` | 姿態估測工作台的韌體端：200 Hz 同時執行 EKF 與 Mahony，經 USART1（921600 baud 二進位協議）把每一筆原始量測與輸出送給 `tools/ahrs_bench` 的 Web UI，接受主機修改參數、重設與 Mahony 靜置校正；OLED 顯示兩者的姿態與連線統計 | `12_key_buzzer`、`21_oled`、`22_imu` | 執行 `tools/ahrs_bench/server.py` 後開啟網頁；長按按鍵兩者重設。USART1 不再是文字主控台 |
| `oled_menu` | OLED 菜單：狀態頁（電池、開機時間、依車頭方向換算的左右輪圈數）、車頭方向設定（存 Flash、重開機生效，見 ADR-0001）、關於頁（硬體版本、IMU、UID） | `12_key_buzzer`、`13_flash_params`、`21_oled`、`23_battery_adc`、`31_encoder` | 使用者按鍵：單擊下一項、長按進入／確認、雙擊返回 |
| `chassis_ctrl` | 生產級底盤整合控制韌體：100 Hz 輪速閉環 PID、差速運動學與軟體車頭方向映射（ADR-0001）、wz 陀螺儀閉環（ADR-0002）、高速 DMA 二進位協議（ADR-0003，50 Hz 遙測）、SBUS > 手柄 > ROS2 固定仲裁與斜坡失效保護（ADR-0004）、OLED 即時儀表與電池監控 | `11_led`、`12_key_buzzer`、`13_flash_params`、`21_oled`、`22_imu`、`23_battery_adc`、`31_encoder`、`32_motor`、`41_sbus` | 接上電池、SBUS 接收機或經 USART1 連接 ROS 2 上位機。OLED 顯示狀態與速度，支援平滑減速失效保護 |

## 建置

新環境先執行 `tools/setup.sh`：檢查並安裝工具鏈、submodule、pyserial、串口權限等，最後會編譯全部範例。

手動準備時需要 `arm-none-eabi-gcc`、CMake 3.20 以上與 Ninja。HAL 與 CMSIS 以 git submodule 放在 `third_party/`：

```bash
git submodule update --init --depth 1
```

```bash
cmake -S examples -B examples/build -G Ninja
```

```bash
cmake --build examples/build
```

輸出在 `examples/build/<範例>.{elf,bin,hex}`。

## 燒錄與監看

`tools/c50c.py` 會自動找 STM32CubeProgrammer 的 `STM32_Programmer_CLI`，也可以用 `STM32_PROGRAMMER_CLI` 環境變數指定。

經 Type-C 串口 1 燒錄（板子以 DTR/RTS 自動進入內建 bootloader）：

```bash
tools/c50c.py flash 01_board_info
```

經 SWD 燒錄：

```bash
tools/c50c.py flash 01_board_info --swd
```

開啟主控台並重設板子，從頭看測試輸出：

```bash
tools/c50c.py monitor --reset
```

板子上有兩顆 CH9102F（USART1 與 USART3），同時接上時要用 `--port` 指定 USART1 那一個；`tools/c50c.py ports` 會列出所有埠。主控台一定要以 RTS、DTR 都無效的狀態開啟，否則自動下載電路可能把板子重設進 bootloader；`monitor` 已經處理好，改用其他終端機時要自行關閉 RTS/DTR。

## 結構

- `common/`：範例共用的板級支援。
  - `board.h`：所有腳位與板級常數，來源是原廠資料的資源分配表與原廠代碼。
  - `board_id.c`：判斷硬體版本與 IMU 型號。
  - `console.c`：USART1 主控台與 `printf`。
  - `report.c`：PASS/FAIL 回報格式。
  - `soft_i2c.c`：軟體 I2C。IMU 的 SCL/SDA 接法與硬體 I2C1 相反，只能用軟體 I2C。
  - `oled.c`：SSD1306 OLED 驅動（GPIO 模擬 SPI）與 5×7 字型。
  - `key.c`：使用者按鍵事件（去彈跳、單擊、雙擊、長按）。
  - `battery_monitor.c`：電池電壓濾波與電量等級判斷（遲滯與持續時間）。
  - `imu.c`：ICM-20948 加速度計與陀螺儀驅動（設定同 `22_imu`，輸出率 225 Hz）。
  - `ctrl_tick.c`：TIM6 定頻控制中斷，量測回呼執行時間與逾時次數。
  - `gyro_cal.c`：陀螺儀零偏校正、靜止偵測與靜止時的零偏追蹤。
  - `ahrs.c`：Mahony 姿態濾波（四元數，K<sub>p</sub>、K<sub>i</sub> 可在執行期修改），輸出 REP-103 尤拉角。
  - `ekf_ahrs.c`：乘性誤差狀態 EKF（姿態與三軸零偏、零角速度量測、自適應加速度權重、1σ 輸出）；參數可在執行期修改，並保留最近一步的內部量（創新量、S、K）供 `ahrs_bench` 視覺化。
  - `frame.c`：串口訊框（COBS 分幀、CRC16），純計算，主機端用同一套規則。
  - `gauge.c`：OLED 儀表（航向儀、人工地平儀）；`oled.c` 另有直線、圓、三角形等繪圖功能。
  - `settings.c`：存在 Flash sector 7 的控制板設定（車頭方向）。`13_flash_params` 會清掉這個 sector。
- `bringup/NN_名稱/`：功能驗證，`NN` 的十位數是依賴層級。新增時建立目錄，並在 `CMakeLists.txt` 依層級順序加一行 `c50c_example(bringup/NN_名稱)`。
- `demos/名稱/`：範例程序；在 `CMakeLists.txt` 加一行 `c50c_example(demos/名稱)`。
- 韌體名稱取最後一段目錄名（燒錄時用 `tools/c50c.py flash 21_oled`），所以 `bringup/` 與 `demos/` 之間不能重名。

## 已知限制

- 原廠代碼把硬體分成 V1.0、V1.1、V1.2，與 PCB 絲印的 V4.x 沒有公開對照；`01_board_info` 只回報原廠代碼的名稱。
- LED 為高電位點亮，且 PE6 = 紅、PB15 = 綠，與原廠代碼的命名相反（V1.1 實機以 `11_led` 確認）。舊版 V1.0 的 LED 在不同腳位，這些範例不支援 V1.0。
