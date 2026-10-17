# 韌體系統

支援 ROS2 的開源 MCU 韌體，用同一套系統驅動多種載具與 frame。MCU 負責即時的狀態估計與低階控制，可脫離上位機獨立運作；全域定位與導航屬於上層 ROS2 應用。

## 載具與硬體

**載具（Vehicle）**：
由韌體控制的整台機器，例如 UGV、USV、UUV。
_Avoid_: 車、機器人、平台

**Frame**：
載具的運動構型，決定致動器如何影響車體運動，例如差速、阿克曼、麥輪、全向、推進器陣列。
_Avoid_: 車型、CarType

**控制板（Board）**：
執行韌體的硬體板卡，例如 C50C。
_Avoid_: 主板、target

**上位機（Companion Computer）**：
與 MCU 連線、執行 ROS2 的電腦，例如 Raspberry Pi 5。
_Avoid_: 主機、Pi、host

## 運作

**獨立運作（Standalone）**：
沒有上位機時，MCU 僅靠自身估測、控制與遙控輸入維持載具運作的狀態。
_Avoid_: 離線模式

**Setpoint**：
送給 MCU 的控制目標，分為三個層級：車體速度、姿態/深度、致動器直出。
_Avoid_: 指令、命令

**指令仲裁（Command Arbitration）**：
MCU 依模式與安全狀態，決定哪個來源的 setpoint 有控制權。
_Avoid_: 優先權切換

## 通訊

**匯流排（Bus）**：
MCU 內部模組之間交換資料的發布/訂閱機制，是韌體的骨幹。
_Avoid_: uORB、message queue

**Topic**：
匯流排上有型別、有名稱的一條資料流。同一個 Topic 可以有多個實例，例如兩顆 IMU。

**Work Queue**：
依頻率層級劃分的執行單位，一個 Work Queue 對應一個 RTOS task。
_Avoid_: 執行緒、thread

**工作項目（Work Item）**：
掛在 Work Queue 上執行的模組，由計時器或 Topic 發布觸發。
_Avoid_: task、job

**Link**：
MCU 與外部系統之間的一條通訊連線，由 Topic 映射、協議模組與傳輸組成。
_Avoid_: 通道、interface

**協議模組（Protocol）**：
Link 中負責把 Topic 編碼成特定協議的部分，例如自訂協議、XRCE-DDS、MAVLink。
_Avoid_: driver

**傳輸（Transport）**：
Link 中承載位元組的實體或邏輯通道，例如 UART、CAN、USB。
_Avoid_: 介面、port

**Topic 映射（Topic Map）**：
宣告哪些 Topic 經由某條 Link 對外、方向與頻率的設定。

**Schema**：
Topic 與訊息型別的唯一定義來源，MCU、Bridge 與工具的程式碼都由它產生。

**Bridge**：
上位機端把 Link 轉成 ROS2 topic、service 與 parameter 的節點。
_Avoid_: driver node、agent
