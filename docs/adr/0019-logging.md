# 日誌採 ULog，後端可替換，反壓在日誌端完成

**格式：ULog**

- 不必自己寫工具：pyulog、PlotJuggler、Lichtblick 都能直接開啟 `.ulg`。
- 與既有決策吻合：每則訊息的第一個欄位是 `timestamp`（ADR 0008），資料採緊密排列的 little-endian（ADR 0006），所以日誌可以直接寫入 Link 的 payload，不必重新編碼。產生器從 Schema 產生 ULog 的格式定義。
- 會用到的訊息類型：`P`（參數）、`I`（板子、硬體版本、韌體版本、Schema 雜湊）、`L`（文字訊息，同時轉送到 ROS2 的 `/rosout`）、`O`（掉資料）、`S`（同步）。
- 要記錄的 Topic 必須是固定長度。
- Bridge 會記錄 MCU 與上位機之間的時鐘偏差，讓 ULog 可以和 rosbag2 錄下的 MCAP 在 Lichtblick 中對齊。

**後端**

所有後端都實作 `LogSink` 虛擬介面，可以同時啟用多個：

- SD 卡：FatFs，每個工作階段一個 `.ulg` 檔。
- SPI NOR Flash：以區塊附加寫入，不建檔案系統。
- 串流到上位機：由 Bridge 存檔。
- RAM 環形緩衝：只在事件發生時，分批寫入內部 Flash 的事件日誌磁區（ADR 0020）。
- 序列埠記錄器。

板級 TOML 宣告儲存裝置，產生 `BOARD_HAS_*` Kconfig 符號，所以要編進哪些後端由 Kconfig 決定。實際啟用哪些由 bitmask 參數 `LOG_BACKEND` 決定，預設值來自板級定義。C50C 沒有板載儲存，所以預設用串流，但這只是它的板級預設值。

**記錄什麼**

- 參數 `LOG_PROFILE` 是 bitmask，可組合以下設定檔（概念同 PX4 的 `SDLOG_PROFILE`）：預設、估測回放、系統識別、高頻控制、除錯。
- 每個設定檔以 TOML 定義記錄哪些 Topic、各自的頻率。
- `LOG_MODE` 決定何時記錄：一直記錄、arming 時記錄、事件時記錄。
- 產生器估算每個設定檔需要的頻寬，開機時再依實際鮑率檢查一次。

**高頻資料與反壓**

- 需要全頻率記錄的 Topic 必須用 `@queue` 或批次訊息，否則產生器報錯。
- 批次訊息照 PX4 的 `SensorGyroFifo`：`int16[32]` 加 `samples`、`dt`、`scale`。湊滿 N 個樣本或超過 T 毫秒，任一條件先到就發布。控制鏈仍然訂閱逐樣本的 Topic。
- 反壓全部在日誌端完成，產生資料的模組完全不知道日誌存在：
  - 日誌模組依訂閱的世代號碼偵測漏讀，寫入 `O` 訊息。
  - 寫入環形緩衝前，先以非阻塞的 `try_reserve` 預留空間，失敗就跳過並計數。
  - 負載過高時，依設定檔的優先級降級。

## Considered Options

- **由產生資料的模組呼叫 `logger_has_headroom()`**：不採用。這會讓高優先級的驅動依賴日誌模組，破壞匯流排解耦，也把判斷放進 `wq:ctrl` 的熱路徑。
- **Betaflight 的壓縮格式、自訂格式、在 MCU 上產生 MCAP**：不採用。前兩者需要專用工具；MCAP 有索引與壓縮區塊，對 MCU 太複雜，交給上位機的 rosbag2 即可。
