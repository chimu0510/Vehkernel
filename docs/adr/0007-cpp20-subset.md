# 韌體以 C++20 子集撰寫

多載具、多 Frame、多板是本項目的核心，需要大量介面抽象，例如運動學模型、HAL backend。同樣以多載具為目標的 PX4 和 ArduPilot 都選了 C++。我們採用 C++20，但限定子集：

- 不用 exception、不用 RTTI。
- 初始化完成後不做動態配置，也不用會配置記憶體的 STL 容器。
- ST HAL 與 FreeRTOS 仍是 C，透過 `extern "C"` 呼叫。

C++ 的另一個好處是產生器可以產出有型別的 Topic，例如 `Subscription<BatteryStatus>`，整條 Schema 鏈都受編譯器檢查。

## Considered Options

- **C11**（Betaflight、原廠 WHEELTEC 的作法）：不採用。多態抽象只能靠函式指標表與巨集，匯流排 API 也只能用 `void*` 加大小，沒有型別安全。
