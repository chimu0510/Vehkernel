# ROS2-UGV

以 WHEELTEC C50C 控制板為下位機的 ROS2 無人地面載具（UGV）項目。

## Language

### 硬體

**C50X 系列**:
WHEELTEC 的控制板產品系列，原廠代碼以此系列命名。
_Avoid_: 用 C50X 指稱本項目使用的板子

**C50C 控制板**:
本項目使用的 C50X 系列控制板（V4.0–V4.2），主控為 STM32F407VET6。
_Avoid_: STM32F407 控制板、C50X 板、主板

### 項目內容

**範例**:
針對 C50C 控制板單一功能的獨立程序，同時用來驗證該功能是否正常。
_Avoid_: demo、sample、測試程序

**原廠資料**:
WHEELTEC 提供的 C50C 相關資料，包含原廠代碼、PCB 原理圖與各 IC 資料手冊。
_Avoid_: source、源碼

**原廠代碼**:
原廠資料中 WHEELTEC 提供的 C50X 系列韌體代碼。
_Avoid_: 源碼、source code、本項目代碼

**擴充資源**:
體積大、不隨 repo 提交，由使用者自行從雲端下載的檔案，例如車款模型。
_Avoid_: assets、外部依賴
