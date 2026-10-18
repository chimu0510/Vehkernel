# Vehkernel

支援 ROS2 的開源 MCU 韌體，用同一套系統驅動多種載具與 frame。MCU 負責即時的狀態估計、低階控制與安全，可脫離上位機獨立運作；全域定位與導航屬於上層 ROS2 應用。

## 載具與硬體

**載具（Vehicle）**：
由韌體控制的整台機器，例如 UGV、USV、UUV。
_Avoid_: 車、機器人、平台

**載具類型（Vehicle Type）**：
載具運作的環境：地面、水面或水下，與 Frame 互相獨立。決定是否啟用深度控制，以及安全行為等參數的預設值。
_Avoid_: 載具種類、Frame Class

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

**主要估測器（Primary Estimator）**：
由參數選定、發布到標準狀態 Topic 的估測器。同時執行的其他估測器發布到診斷 Topic，用來比較演算法，主要估測器不健康時可以切換過去當備援。每則狀態訊息都帶估測器 ID。
_Avoid_: 主 EKF

**車體速度量測（Body Velocity Measurement）**：
車體座標下的速度量測，帶來源 ID 與共變異數。來源可以是載具模型的輪速換算、DVL 這類測速感測器，或上位機送下的 VIO 速度。
_Avoid_: 輪速里程計、wheel odom

**局部里程計（Local Odometry）**：
估測器融合車體速度量測與航向推算出的局部位姿與速度，發布為 `vehicle_local_odometry`。不綁定任何一種感測器。
_Avoid_: 輪式里程計

**外部估測器（External Estimator）**：
把上位機送下的位姿（例如 VIO）換算成 MCU 時間後發布的估測器，可以選為主要估測器，上位機斷線時切回 MCU 自己的估測器。
_Avoid_: VIO 直通

**車體運動需求（Body Motion Demand）**：
控制鏈交給載具模型的需求：輪式載具用 twist，推進器載具用 wrench。
_Avoid_: 車體指令

**致動器（Actuator）**：
把控制輸出轉成運動的元件，例如馬達、舵機、推進器。

**致動器控制（Actuator Control）**：
控制鏈的最後一層，把各致動器的需求轉成 PWM 等輸出，例如輪速 PI 加編碼器回授。
_Avoid_: 馬達驅動（驅動指驅動層的裝置驅動）

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

**周邊匯流排（Peripheral Bus）**：
MCU 與板上裝置之間的硬體匯流排，例如 I2C、SPI，包含以 GPIO 模擬的軟體實作。
_Avoid_: 匯流排（單獨使用時指內部的發布/訂閱匯流排）

**遙控器（Remote Control）**：
操作者直接操控載具的輸入裝置，例如 C50C V1.1 的 USB 手柄或 SBUS 接收機。不受模式限制，隨時可以接管或急停。
_Avoid_: 手把

**地面站（Ground Station）**：
在載具外、經由 Link 監看或操控載具的工具，例如 QGroundControl。不是上位機。

**上位機（Companion Computer）**：
與 MCU 連線、執行 ROS2 的電腦，例如 Raspberry Pi 5。
_Avoid_: 主機（單獨使用時）、Pi、host

**開發主機（Development Host）**：
執行建置、主機單元測試、SITL 與燒錄工具的電腦，例如開發者的電腦或 CI runner。不是上位機。
_Avoid_: PC、電腦、host（單獨使用時）

## 運作

**獨立運作（Standalone）**：
沒有上位機時，MCU 僅靠自身估測、控制與遙控輸入維持載具運作的狀態。
_Avoid_: 離線模式

**Setpoint**：
送給 MCU 的控制目標，分為三個層級：車體速度、姿態/深度、致動器直出。
_Avoid_: 指令、命令

**模式（Mode）**：
決定哪個來源有控制權，以及它可以使用哪些 setpoint 層級的運作狀態，例如上位機模式、手動模式。
_Avoid_: 致動器直出模式（致動器直出是 setpoint 層級）

**Arming**：
允許致動器輸出的狀態。armed 時致動器依控制鏈輸出；disarmed 時致動器不輸出，Flash 抹除這類會卡住 CPU 的操作只在這時進行。
_Avoid_: 解鎖、上鎖

**安全行為（Failsafe Action）**：
有控制權的來源逾時或失效時，載具自動採取的動作，例如停車、懸停或上浮。預設值依載具類型而定。
_Avoid_: failsafe 模式（安全行為不是一種模式）

**Setpoint 仲裁（Setpoint Arbitration）**：
MCU 依模式與安全狀態，決定哪個來源的 setpoint 有控制權。
_Avoid_: 指令仲裁、優先權切換

## 日誌

**日誌後端（Log Sink）**：
日誌寫入的目的地，例如 SD 卡、SPI NOR Flash、串流到上位機、RAM 環形緩衝、序列埠記錄器。
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

**板級預設（Board Defaults）**：
板級定義提供的參數預設值與 Kconfig 預設值，在參數預設值分層中位於參數定義之上、Frame 預設組之下。

**時序預設組（Timing Preset）**：
觸發來源、基準頻率、分頻與取樣模式的一組預設值，由板級定義提供，Frame 預設組可以覆寫。
_Avoid_: loop rate

**Frame 預設組（Frame Preset）**：
套用到特定 Frame 或車款的一組參數預設值。
_Avoid_: airframe、車型參數

## 通訊

**匯流排（Bus）**：
MCU 內部模組之間交換資料的發布/訂閱機制，是韌體的骨幹。
_Avoid_: uORB、message queue；I2C、SPI 這類硬體匯流排稱為周邊匯流排

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

**Schema 雜湊（Schema Hash）**：
產生器為每個訊息型別算出的雜湊。連線握手時逐一比對，任何一個不一致就拒絕連線。協議本身的版本則另外協商。

**Bridge**：
上位機端把 Link 轉成 ROS2 topic、service 與 parameter 的節點。
_Avoid_: driver node、agent

## 測試

**Bring-up**：
新板子或新接線第一次上電時，確認接線、對應關係與方向是否正確的測試，例如讀 IMU 的 WHO_AM_I、確認馬達與編碼器的對應。
_Avoid_: 開機測試

**測試模式（Test Mode）**：
一種模式：控制權交給開發主機上的測試執行器，只接受致動器直出層級，用來執行 bring-up 案例並透過 Link 回報結果。會轉動輪子的案例，須先確認車輪離地並限制輸出功率。

## 軟體分層

軟體分層用名稱稱呼，不編號；L0–L5 專指測試分層（ADR 0018）。依賴方向由上往下：模組層 → 驅動層 → 介面層 → 平台層，可以跳過驅動層，但不能繞過介面層（ADR 0009）。

**模組層（Module Layer）**：
估測器、控制鏈、載具模型、Link、日誌等功能模組，放在 `modules/`。
_Avoid_: L3、應用層

**驅動層（Driver Layer）**：
板上與外接裝置的驅動，例如 IMU、編碼器，放在 `drivers/`。
_Avoid_: L2

**介面層（Interface Layer）**：
HAL 的契約，以 C++20 concept 與執行時介面定義，放在 `hal/`。
_Avoid_: L1

**平台層（Platform Layer）**：
介面層在特定 MCU 或 SITL 上的實作，放在 `platform/`。
_Avoid_: L0、BSP

**能力（Capability）**：
介面層以具體功能提供的平台相關介面，例如 `PwmOutput`、`QuadratureEncoder`，用來取代高度依賴平台的通用計時器介面。
