# 韌體系統

支援 ROS2 的開源 MCU 韌體，用同一套系統驅動多種載具與 frame。MCU 負責即時的狀態估計與低階控制，可脫離上位機獨立運作；全域定位與導航屬於上層 ROS2 應用。

## 載具與硬體

**載具（Vehicle）**：
由韌體控制的整台機器，例如 UGV、USV、UUV。
_Avoid_: 車、機器人、平台

**Frame**：
載具的運動構型，決定致動器如何影響車體運動，由 Frame Class 與 Frame Type 組成。
_Avoid_: 車型、CarType

**Frame Class**：
依分配的數學模型劃分的 Frame 類別，例如 WHEELED、ACKERMANN、THRUSTER。
_Avoid_: 載具種類

**Frame Type**：
Frame Class 底下具名的幾何配置，例如 MECANUM_4、DIFF_2WD。

**載具模型（Vehicle Model）**：
把車體運動需求分配成各致動器需求的數學模型，由 Frame Class 決定。
_Avoid_: mixer、運動學解算

**控制鏈（Control Chain）**：
從 setpoint 到致動器輸出、所有 Frame 共用的一串控制層。
_Avoid_: 控制迴圈

**估測器（Estimator）**：
由一個濾波演算法加上若干數學模型組成、發布標準狀態 Topic 的模組。
_Avoid_: AHRS、EKF（這兩個是特定的估測器）

**控制板（Board）**：
執行韌體的硬體板卡，例如 C50C。
_Avoid_: 主板、target

**硬體版本（Board Revision）**：
同一款控制板的硬體修訂，名稱以原廠韌體回報的為準，例如 C50C V1.1。
_Avoid_: V4.1（與原廠韌體名稱的對應未經確認）

**替代配置（Alt Config）**：
板級定義中，一支腳位在特定 Frame 下的另一種用途，開機時依參數選用。
_Avoid_: AF、複用（AF 是 STM32 腳位的 alternate function 編號，是不同的概念）

**MCU 資料庫（MCU Database）**：
某款 MCU 的腳位 AF 對照與 DMA 請求對照，產生器據此驗證板級定義。
_Avoid_: pinmap、chip table

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

## 日誌

**日誌後端（Log Sink）**：
日誌寫入的目的地，例如 SD 卡、SPI Flash、串流到上位機、事件記錄器。
_Avoid_: logger device

**日誌設定檔（Log Profile）**：
一組要記錄的 Topic 和各自的記錄頻率，可以組合使用。
_Avoid_: 日誌等級

**批次訊息（Batch Message）**：
在一則訊息中裝入多個連續樣本的 Topic，用於高頻資料。
_Avoid_: FIFO topic

## 參數

**參數（Parameter）**：
可以在執行時調整、斷電後仍保留的設定值，名稱不超過 16 個字元。
_Avoid_: 設定、config

**參數群組（Parameter Group）**：
同一個模組的參數集合，在 MCU 上是一個結構體。
_Avoid_: PG

**Frame 預設組（Frame Preset）**：
套用到特定 Frame 或車款的一組參數預設值。
_Avoid_: airframe、車型參數

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
