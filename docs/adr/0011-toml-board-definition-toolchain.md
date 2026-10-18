# 板級定義採用 TOML，並搭配 MCU 資料庫與 DMA 求解器

ArduPilot hwdef 真正的價值是 MCU 資料庫、腳位驗證與 DMA 求解器，`.dat` 格式本身不是重點。PX4 的作法則是在 `board_config.h` 中手動組合腳位，並在 `timer_config.cpp` 中手填 DMA stream 與 channel，衝突完全不檢查。我們取 ArduPilot 的工具鏈，但把格式改成 TOML，與參數定義共用同一種格式。

- **一行一個腳位**：每個腳位寫成一個 inline table，例如 `{ pin = "PA9", role = "SERIAL1_TX", func = "USART1_TX" }`。TOML 1.0 要求 inline table 寫在同一行，所以格式本身就保證了一行一個腳位，可以和原理圖逐行比對。產生器只能抓到「不可能」的錯誤，抓不到「合法但錯誤」的配置，例如編碼器 A/B 接反，所以仍然需要比對原理圖。
- **板上裝置**：除了腳位，板級定義也宣告板上裝置與它們的安裝方向、周邊匯流排類型（ADR-0015）、儲存裝置（ADR-0019）與 Flash 配置（ADR-0020）。
- **連接埠**：板級定義只宣告邏輯連接埠（`SERIAL1`、`SERIAL2`⋯）對應哪個周邊與哪些腳位，不決定用途。用途在執行時由參數 `SERIALn_FUNCTION` 指派，例如 Link 或遙控接收機，預設值來自板級預設參數（概念同 ArduPilot 的 `SERIALn_PROTOCOL`）。換板子時，同一個邏輯連接埠可以對到不同的 UART，參數不必改。
- **繼承**：`extends` 與 `remove` 由產生器實作，取代 hwdef 的 `include` / `undef`。替代配置寫成腳位的 `alt` 欄位。
- **MCU 資料庫**：AF 表、DMA 請求對照表與 Flash 磁區表，從 STM32Cube 資料庫（可經由 stm32-data 專案）離線轉成 JSON，連同資料來源一起 commit。建置時只讀這份 JSON，不需要網路，結果可重現。
- **DMA 求解器**：F4 沒有 DMAMUX，每個請求只能用固定幾組 stream/channel，所以用回溯法求解。結果必須是確定的，並輸出指派報告；可用註記指定優先級、不使用 DMA 或允許共用。
- **產出**：有型別的腳位常數 `board.hpp`、時脈、DMA 指派、周邊實例表、板級預設參數（含時序預設組，ADR-0015）、`BOARD_HAS_*` 這類 Kconfig 符號（ADR-0017）、連結腳本的記憶體區段與 Flash 預算（ADR-0020）、JSON dump。產生器也會檢查每一組替代配置都沒有衝突。

## Considered Options

- **仿 ArduPilot `hwdef.dat` 的行式格式**：曾經採用，後來放棄。真正有價值的是背後的工具鏈，自訂語法還得自己寫解析器；TOML 的 inline table 一樣能做到一行一個腳位。
- **YAML 板級定義**：不採用。它用樹狀結構描述扁平的表格，篇幅膨脹，難以和原理圖逐行比對。
