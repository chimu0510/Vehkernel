# AGENTS.md

## Project structure

- `examples/`: **範例**。C50C 控制板各功能的範例程序，同時用來驗證功能是否正常。
- `tools/`: 項目的腳本與工具（build、scripts 等）。
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
