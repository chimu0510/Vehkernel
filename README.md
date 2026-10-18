# Vehkernel

**Vehkernel**（Vehicle + Kernel）是一套支援 ROS2 的開源 MCU 韌體系統，定位為任何載具的通用骨架：同一套系統可以驅動多種載具與 Frame。MCU 負責即時的狀態估計、低階控制與安全，沒有上位機也能獨立運作；全域定位與導航由上層的 ROS2 應用負責。首要測試平台是 UGV。

**目前狀態：架構設計階段。** 所有決策都記錄在 [`docs/adr/`](docs/adr/)，用詞以 [`GLOSSARY.md`](GLOSSARY.md) 為準，還沒有可以執行的程式碼。

## 目標硬體

| 項目 | 目前支援 |
|---|---|
| MCU | STM32F407VET6 |
| 控制板 | C50C V1.1（ICM20948、USB 手柄，沒有 485） |
| 上位機 | Raspberry Pi 5（ROS2） |

其他硬體版本或控制板，等需要時再新增各自的板級定義（[ADR 0010](docs/adr/0010-board-revision-policy.md)）。

## 架構重點

- **定位**：MCU 是上位機的即時協處理器與安全守門員，不是自駕儀（[0001](docs/adr/0001-mcu-responsibility-boundary.md)）。
- **通訊**：
  - 內部以發布/訂閱匯流排為骨幹，ROS2 只是其中一條 Link；Link 由 Topic 映射、協議模組、傳輸組成，可以替換（[0002](docs/adr/0002-bus-backbone-and-pluggable-links.md)）。
  - 訊息由單一份 Schema 定義，程式碼自動產生（[0006](docs/adr/0006-schema-and-wire-encoding.md)）。
- **ROS2 介面**：
  - 提供載具級的 topic API，setpoint 分三個層級；ros2_control 是選配（[0003](docs/adr/0003-vehicle-level-ros2-api.md)）。
  - 控制權由模式決定，遙控器隨時可以接管（[0005](docs/adr/0005-setpoint-arbitration.md)）。
- **執行環境**：FreeRTOS、C++20 子集，Work Queue 依頻率層級劃分（[0004](docs/adr/0004-freertos.md)、[0007](docs/adr/0007-cpp20-subset.md)、[0008](docs/adr/0008-bus-and-work-queues.md)）。
- **硬體適配**：
  - 板級定義用 TOML 撰寫，產生器根據 MCU 資料庫驗證腳位、求解 DMA 分配（[0011](docs/adr/0011-toml-board-definition-toolchain.md)）。
  - HAL 依「在什麼時候做選擇」決定多型的形式（[0009](docs/adr/0009-board-definition-and-hal.md)）。
- **配置**：
  - Kconfig 決定韌體編進哪些功能，參數決定執行時啟用哪一個（[0017](docs/adr/0017-kconfig-build-configuration.md)、[0012](docs/adr/0012-parameter-system.md)）。
  - 第三方套件可以使用，但必須固定版本（[0016](docs/adr/0016-dependency-policy.md)）。
- **控制與估測**：
  - 所有 Frame 共用一條控制鏈，差異只在載具模型；Frame 分成 Class 與 Type 兩層，載具類型（地面、水面、水下）另外決定深度控制與安全行為的預設值（[0013](docs/adr/0013-control-chain-and-vehicle-models.md)）。
  - 估測器可以替換，數學元件拆成獨立的函式庫（[0014](docs/adr/0014-estimation-architecture.md)）。
  - 時序拆成幾個可以選擇的維度（[0015](docs/adr/0015-selectable-timing.md)）。
- **品質與觀測**：
  - 測試分六層，SITL 跑的是同一套 RTOS（[0018](docs/adr/0018-test-layers-and-sitl.md)）。
  - 日誌採用 ULog 格式，後端可以替換（[0019](docs/adr/0019-logging.md)）。

## 目錄結構

以下是初始骨架，實作時可以調整。

| 目錄 | 內容 | 相關 ADR |
|---|---|---|
| `boards/` | 板級定義（TOML）與板級 defconfig | 0010、0011、0017 |
| `mcu/` | MCU 資料庫：AF 對照、DMA 請求對照（JSON） | 0011 |
| `schema/msg/` | Topic 的 Schema（`.msg`） | 0006 |
| `cmake/` | 工具鏈、Kconfig 整合、產生器呼叫 | 0017 |
| `hal/` | 介面層：HAL 介面（C++20 concept 與執行時介面） | 0009 |
| `platform/stm32f4/` | 平台層：STM32F4 實作（ST LL） | 0009 |
| `platform/sitl/` | SITL 與測試用的平台實作 | 0009、0018 |
| `drivers/` | 驅動層：裝置驅動 | 0009 |
| `core/` | 匯流排、Work Queue、參數執行時、時序監看 | 0008、0012、0015 |
| `modules/` | 模組層：估測器、控制鏈、載具模型、Link、日誌 | 0002、0013、0014、0019 |
| `lib/estimation/` | 估測數學元件：`models/`、`filters/`、`math/` | 0014 |
| `tools/generators/` | Schema（含 ULog 格式定義）、板級、參數、日誌設定檔產生器 | 0006、0011、0012、0019 |
| `bridge/` | 上位機端的 ROS2 Bridge | 0002、0003 |
| `sim/` | 內建模擬器與外部模擬器介面 | 0018 |
| `test/` | `unit/`、`replay/`、`sitl/`、`hil/` | 0018 |

## 授權

GNU GPL 第 3 版或任何更新的版本（SPDX：`GPL-3.0-or-later`），全文見 [LICENSE](LICENSE)。
