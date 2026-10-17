# 板級定義改用 TOML，並搭配 MCU 資料庫與 DMA 求解器

本 ADR 取代 ADR 0009 的板級定義格式；ADR 0009 中 HAL 分層的部分仍然有效。

ArduPilot hwdef 真正的價值是 MCU 資料庫、腳位驗證與 DMA 求解器，`.dat` 格式本身不是重點。PX4 的作法則是在 `board_config.h` 中手動組合腳位，並在 `timer_config.cpp` 中手填 DMA stream 與 channel，衝突完全不檢查。我們取 ArduPilot 的工具鏈，但把格式改成 TOML，與參數定義共用同一種格式。

- **一行一個腳位**：每個腳位寫成一個 inline table，例如 `{ pin = "PA9", role = "LINK1_TX", func = "USART1_TX" }`。TOML 1.0 要求 inline table 寫在同一行，所以格式本身就保證了一行一個腳位，可以和原理圖逐行比對。產生器只能抓到「不可能」的錯誤，抓不到「合法但錯誤」的配置，例如編碼器 A/B 接反，所以仍然需要比對原理圖。
- **繼承**：`extends` 與 `remove` 由產生器實作，取代 hwdef 的 `include` / `undef`。替代配置寫成腳位的 `alt` 欄位。
- **MCU 資料庫**：AF 表與 DMA 請求對照表，從 STM32Cube 資料庫（可經由 stm32-data 專案）離線轉成 JSON，連同資料來源一起 commit。建置時只讀這份 JSON，不需要網路，結果可重現。
- **DMA 求解器**：F4 沒有 DMAMUX，每個請求只能用固定幾組 stream/channel，所以用回溯法求解。結果必須是確定的，並輸出指派報告；可用註記指定優先級、不使用 DMA 或允許共用。
- **產出**：有型別的腳位常數 `board.hpp`、時脈、DMA 指派、周邊實例表、板級預設參數、JSON dump。產生器也會檢查每一組替代配置都沒有衝突。
