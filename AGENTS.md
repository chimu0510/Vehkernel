# AGENTS.md

## Project structure

- `examples/`: **範例**。C50C 控制板各功能的範例程序，同時用來驗證功能是否正常（建置與燒錄見 `examples/README.md`）。
  - `common/board.h`: C50C 腳位與板級常數的唯一出處。
- `tools/`: 項目的腳本與工具（build、scripts 等）。
  - `cmake/arm-none-eabi.cmake`: STM32F407 交叉編譯工具鏈。
  - `c50c.py`: C50C 燒錄（串口或 SWD）、重設與主控台。
- `third_party/`: git submodule，ST 官方的 STM32F4 HAL、CMSIS Device 與 CMSIS Core，版本已釘住。
- `vendor/`: **原廠資料**。C50C 控制板的原廠代碼、PCB 原理圖與各 IC 資料手冊。目前不提交到 git。
- `extensions/`: **擴充資源**。不提交到 git，使用者自行從雲端下載，下載方式見 `extensions/README.md`。
- `ros2_ws/`: ROS2 workspace（含 arm64 交叉編譯腳本，見 `ros2_ws/scripts/README.md`）。
  - `src/`: 部署到 Raspberry Pi 5 的套件，交叉編譯只編譯這裡。
  - `host_tools/`: 僅在開發主機使用的工具套件（如 `ugv_model_viewer`），不部署到目標。

項目代碼不得依賴原廠資料；需要原廠的大型檔案（如車款模型）時，複製到擴充資源中使用。

術語定義見 `CONTEXT.md`。

## Agent skills

### Issue tracker

Issues are tracked in GitHub Issues (gugugaga-hehe/ROS2-UGV) via the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

Default vocabulary: needs-triage, needs-info, ready-for-agent, ready-for-human, wontfix. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: one `CONTEXT.md` + `docs/adr/` at the repo root. See `docs/agents/domain.md`.
