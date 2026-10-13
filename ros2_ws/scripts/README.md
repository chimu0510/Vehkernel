# ROS 2 ARM64 交叉編譯工具集

本目錄存放用於在 x86_64 主機環境下為 aarch64 (ARM64) 目標設備（例如 RK3566、樹莓派、Jetson）交叉編譯 ROS 2 Jazzy 軟體包的核心腳本與配套文件。

所有腳本已完成**無絕對路徑**改造，採用動態相對路徑解析，支援在任意工作目錄或任意主機路徑下運行。

---

## 一、 目錄結構規範

交叉編譯系統預期之工作空間結構如下：

```text
ros2_ws/                                 # 工作空間根目錄 (WORKSPACE_DIR)
├── scripts/                             # 本腳本目錄 (SCRIPT_DIR)
│   ├── cross_compile.sh                 # 交叉編譯與自動遠程部署主腳本
│   ├── Dockerfile                       # 目標環境鏡像定義 (用於依賴解析與生成 Sysroot)
│   ├── fix_sysroot_paths.sh             # Sysroot 內部 CMake 絕對路徑修復腳本
│   ├── setup_qemu_environment.sh        # QEMU / Docker / Sysroot 初始化腳本
│   ├── toolchain.cmake                  # CMake aarch64 交叉編譯工具鏈配置
│   └── README.md                        # 本說明文件
├── src/                                 # 目標源碼編譯目錄 (scripts 的上一層目錄中的 src)
│   └── <your_ros2_packages>             # 待編譯之 ROS 2 功能包
├── ubuntu_arm64/                        # 導出之 ARM64 目標系統 Sysroot (由 setup 腳本生成或軟連結)
├── build/                               # colcon build 中間生成目錄
└── install/                             # 交叉編譯完成之 ARM64 二進制產物目錄
```

---

## 二、 核心腳本說明

| 腳本 / 文件 | 說明 |
| :--- | :--- |
| `setup_qemu_environment.sh` | 註冊 QEMU binfmt 模擬環境，調用 Docker 構建 ARM64 環境鏡像，自動導出 Sysroot 至 `../ubuntu_arm64`，並調用 `fix_sysroot_paths.sh` 進行規範化。 |
| `Dockerfile` | 以 `ros:jazzy` ARM64 為基礎鏡像，配置清華源，並通過 `rosdep` 自動讀取 `../src` 下所有包的 `package.xml` 並安裝二進制依賴。 |
| `toolchain.cmake` | CMake 交叉編譯工具鏈。自動定位 `aarch64-linux-gnu-gcc/g++`，動態綁定 `${CMAKE_SYSROOT}` 與 `pkg-config`，避免主機路徑污染。 |
| `fix_sysroot_paths.sh` | 遞歸修復導出的 Sysroot 中 hardcoded 之絕對路徑，全數替換為動態 CMake 變數 `${CMAKE_SYSROOT}`。 |
| `cross_compile.sh` | 交叉編譯入口。自動定位工作空間根目錄與 `src` 目錄，配置交叉編譯環境變數，調用 `colcon build --base-paths src` 進行編譯；支援離線打包（`pack`）與 SSH/SCP 自動遠端部署（`dest`），打包完整工作空間（`install/` + `build/` + `log/` + `src/`）。 |

---

## 三、 使用步驟

### 1. 初始化交叉編譯環境 (僅需執行一次)
確保已安裝 Docker 與 sudo 權限（腳本會自動檢查並安裝 `docker-buildx` 與交叉編譯工具鏈），執行：
```bash
cd scripts
./setup_qemu_environment.sh
```
> **提示**：亦可於項目根目錄執行 `./tools/setup.sh --only ros` 一併檢查並安裝 Docker、Buildx 與依賴。
> 若先前已在其他工作目錄生成過 `ubuntu_arm64`，亦可直接在工作空間根目錄建立軟連結：
> ```bash
> ln -s /path/to/existing/ubuntu_arm64 ../ubuntu_arm64
> ./fix_sysroot_paths.sh ../ubuntu_arm64
> ```

### 2. 準備源碼
將欲編譯的 ROS 2 功能包放置於工作空間根目錄下的 `src/` 中：
```bash
# 例如：
mkdir -p ../src
# 將自定義包放入 ../src
```

### 3. 開始交叉編譯
無論當前終端機處於工作空間根目錄還是 `scripts/` 目錄中，皆可直接執行：
```bash
./scripts/cross_compile.sh
# 或在 scripts 目錄下：
./cross_compile.sh
```
編譯完成後，二進制產物將存放於工作空間根目錄下的 `install/` 中。

### 4. 離線打包完整工作空間 (包含 install + build + log + src)
若需生成離線安裝包手動透過隨身碟或手動拷貝到目標設備，可使用 `pack` 指令：
```bash
./scripts/cross_compile.sh pack <目標設備的工作空間路徑>

# 範例 (指定目標設備路徑為 /home/ubuntu/ugv_ws)：
./scripts/cross_compile.sh pack /home/ubuntu/ugv_ws
```
執行後將在工作空間根目錄生成 `ugv_ws.tar.gz`（約 30~35 MB），其內已自動將 `install/` 中的環境配置路徑轉換為目標路徑，解壓即可直接運行。

### 5. 編譯並自動部屬完整工作空間至目標設備
若欲在編譯完成後直接透過網路傳送並在機器人上自動解壓縮，可使用 `dest` 參數：
```bash
./scripts/cross_compile.sh dest <SSH端口> <目標用戶名>@<目標IP>:<目標存放路徑>

# 範例：
./scripts/cross_compile.sh dest 22 ubuntu@192.168.1.100:/home/ubuntu/ugv_ws
```
腳本會自動：
1. 編譯並自動適配遠程目標路徑環境。
2. 打包完整的 `install/` + `build/` + `log/` + `src/` 至 `ugv_ws.tar.gz`。
3. 透過 SSH/SCP 傳輸至目標設備並自動於目標路徑解壓縮。
4. 清除臨時壓縮檔並還原開發機本機路徑。

### 6. 在目標設備上運行驗證
在目標板（如 RK3566 / 樹莓派）的終端機執行：
```bash
cd /home/ubuntu/ugv_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 檢查套件是否正確識別
ros2 pkg list | grep <your_package>
```
