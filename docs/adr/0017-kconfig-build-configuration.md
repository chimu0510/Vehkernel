# 編譯時的配置用 Kconfig 求解，CMake 只負責建置

建置系統採 CMake + `CMakePresets.json` + Ninja，每個建置目標一個 preset，例如 `c50c-v1_1`、`sitl`、`test`。模組與驅動的選擇有依賴（`depends on`）、自動帶入（`select`）與互斥選擇（`choice`）等約束。我們不用 CMake 腳本或板級 TOML 清單自己實作這些，而是採用 Kconfig，透過 kconfiglib 在 CMake 配置階段求解。這和 PX4 的作法相同，PX4 以 `default.px4board` 片段配合 `cmake/kconfig.cmake`。

**規則**

- **Kconfig 只決定韌體裡包含什麼，參數決定開機後啟用哪一個。** 例如 `VEHICLE_MODEL_WHEELED=y` 是 Kconfig，`FRAME_CLASS = WHEELED` 是參數。Kconfig 裡不放任何執行時的選擇。參數的列舉值由產生器依 Kconfig 的結果過濾，沒有編進韌體的選項不會出現。
- **硬體事實只寫在板級 TOML。** 板級產生器把硬體能力輸出成 Kconfig 符號，例如 `BOARD_HAS_CAN`、`BOARD_IMU_BUS_SOFT_I2C`，概念同 Zephyr 從 devicetree 產生的 `DT_HAS_*` 符號。驅動用 `depends on` 依賴這些符號。
- **配置分三層**：MCU 預設 → 板級預設 → 除錯覆寫。每次合併後，Kconfig 都會重新驗證所有約束。一塊板子只產出一個韌體檔（ADR-0009），板子支援的載具模型全部編入，由參數選用；不設依 Frame 或產品的覆寫層。
- **`select` 只用於本身沒有依賴的輔助函式庫**，例如 `MATH_QUATERNION`，其餘情況一律用 `depends on`。原因是 `select` 不會檢查被選項目自己的依賴。
- 所有產生器（Schema、板級、參數，以及 ADR-0019 的 ULog 格式定義與日誌設定檔頻寬估算）都以 CMake custom command 執行，產生的程式碼不 commit。
- 分層規則由 CMake target 的相依關係強制執行（ADR-0009）。
- 每次建置都檢查 Flash、RAM、CCM 用量，超過板級預算就失敗。

## Consequences

- PX4 與 Zephyr 都只產生單一的設定標頭檔，改任何一個選項都會觸發大量重新編譯。Linux 用 `fixdep` 解決這個問題，但 kconfiglib 不附帶這個機制。之後若有需要，可以為每個模組產生只含相關選項的設定標頭檔，第一版先不做。
- 若之後 Flash 不足以容納所有載具模型，再以新的 ADR 引入精簡版韌體，而不是在 Kconfig 裡依 Frame 分版。
