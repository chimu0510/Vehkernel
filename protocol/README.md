# 自訂串口通訊協議 (ADR-0003)

C50C 控制板（STM32F407）與 Raspberry Pi 5 之間的專用二進位串口通訊協議。

## 特性

* **COBS 分幀 (Consistent Overhead Byte Stuffing)**：以 `0x00` 作為訊框邊界，避免資料中出現幀頭引發失步，單一訊框損毀最多掉一幀即可重新同步。
* **CRC16-CCITT 校驗**：多項式 `0x1021`，初值 `0xFFFF`，Little-endian。
* **統一微秒時間戳**：每幀包含 MCU 微秒時間戳，供 ROS 2 上位機進行時鐘偏移估算與感測器時間對齊。
* **單一定義檔自動生成代碼**：由 `protocol_def.yaml` 經 `codegen.py` 自動產出 C、C++ 與 Python 程式碼，確保兩端二進位結構體絕對一致。

## 目錄結構

```text
protocol/
├── protocol_def.yaml       # 協議核心定義 (Schema)
├── codegen.py              # 代碼生成工具 (產出 C、C++ 與 Python)
├── CMakeLists.txt          # CMake 依賴配置
├── include/ugv_protocol/
│   └── frame.h, frame.c    # COBS 與 CRC16 分幀編解碼實作
├── c/
│   └── ugv_protocol.h      # 生成之 C 標頭檔 (供 C50C 韌體引用)
├── cpp/
│   └── ugv_protocol.hpp    # 生成之 C++ 標頭檔 (供 ROS 2 節點引用)
├── ugv_protocol.py         # 生成之 Python 模組 (供測試與調試工具使用)
└── tests/
    ├── test_protocol.py    # Python 單元測試
    └── test_codec.cpp      # C/C++ 編解碼驗證程式
```

## 代碼生成

修改 `protocol_def.yaml` 後，執行：

```bash
./protocol/codegen.py
```

## 執行測試

```bash
python3 protocol/tests/test_protocol.py
```
