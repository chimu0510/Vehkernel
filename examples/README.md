# C50C 控制板範例

每個範例是一個獨立的韌體，只測試一項功能，同時用來驗證該功能是否正常。測試結果從 USART1（Type-C 串口 1，115200 8N1）輸出：

```
==== 05_imu_icm20948 ====
[PASS] who_am_i: 0xEA (expect 0xEA)
[FAIL] gravity: |a| = 0.612 g (expect 0.90..1.10)
[SKIP] buzzer: no answer
RESULT: FAIL (8/9 passed, 1 skipped)
```

最後一行固定以 `RESULT:` 開頭，結束時 LED 亮綠燈表示全部通過、亮紅燈表示有失敗。

## 範例列表

建議依序執行；後面的範例假設前面的已經通過。

| 範例 | 測試內容 | 需要的操作 |
|---|---|---|
| `01_board_info` | MCU 型號與 Flash 大小、8 MHz 晶振、168 MHz 時脈、硬體版本、IMU 型號 | 無 |
| `02_led` | RGB LED 的紅、綠、藍、白、全滅 | 看 LED，在主控台回答 y/n |
| `03_key_buzzer` | 使用者按鍵（PC5）、使能開關（PE4）、蜂鳴器（PD11） | 按一次按鍵、來回切換使能開關、回答是否聽到蜂鳴器；蜂鳴器要插上跳線帽 |
| `04_uart` | USART1 與 USART3 的收發 | 在兩個 Type-C 串口各輸入 `ping` |
| `05_imu_icm20948` | ICM-20948 辨識與設定、靜止時的重力與零偏、磁力計、z 軸角速度 | 先靜置，再水平逆時針轉動車子；之後持續輸出量測值 |
| `06_battery_adc` | VDDA、電池電壓與雜訊；可選與萬用電表比對 | 接上電池；可輸入萬用電表讀數 |

## 建置

需要 `arm-none-eabi-gcc`、CMake 3.20 以上與 Ninja。HAL 與 CMSIS 以 git submodule 放在 `third_party/`：

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
- `NN_名稱/main.c`：各範例；新增範例時建立目錄，並在 `CMakeLists.txt` 加一行 `c50c_example(NN_名稱)`。

## 已知限制

- 原廠代碼把硬體分成 V1.0、V1.1、V1.2，與 PCB 絲印的 V4.x 沒有公開對照；`01_board_info` 只回報原廠代碼的名稱。
- LED 為高電位點亮，且 PE6 = 紅、PB15 = 綠，與原廠代碼的命名相反（V1.1 實機以 `02_led` 確認）。舊版 V1.0 的 LED 在不同腳位，這些範例不支援 V1.0。
