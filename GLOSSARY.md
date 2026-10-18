# Vehkernel

支援 ROS2 的開源 MCU 韌體，用同一套系統驅動多種載具與 Frame。MCU 負責即時估測、低階控制與安全，可脫離上位機獨立運作，全域定位與導航屬於上層的 ROS2 應用。

## Language

### 載具與硬體

**載具（Vehicle）**：
由韌體控制的整台機器，例如 UGV、USV、UUV。
_Avoid_: 車、機器人、平台

**載具類型（Vehicle Type）**：
載具運作的環境：地面、水面或水下，與 Frame 互相獨立。
_Avoid_: 載具種類

**Frame**：
載具的運動構型，由 Frame Class 與 Frame Type 組成。
_Avoid_: 車型、CarType

**Frame Class**：
依分配的數學模型劃分的 Frame 類別，例如 WHEELED、ACKERMANN、THRUSTER。
_Avoid_: 載具種類

**Frame Type**：
Frame Class 底下具名的幾何配置，例如 MECANUM_4、DIFF_2WD。

**載具模型（Vehicle Model）**：
把車體運動需求分配成各致動器需求的數學模型，每個 Frame Class 一個。
_Avoid_: mixer、運動學解算、運動學模型

**控制鏈（Control Chain）**：
從 setpoint 到致動器輸出、所有 Frame 共用的一串控制層。
_Avoid_: 控制迴圈

**估測器（Estimator）**：
由一個濾波演算法加上若干數學模型組成、發布標準狀態 Topic 的模組。
_Avoid_: AHRS、EKF

**主要估測器（Primary Estimator）**：
由參數選定、輸出發布到標準狀態 Topic 的估測器；同時執行的其他估測器只發布到診斷 Topic。
_Avoid_: 主 EKF

**車體速度量測（Body Velocity Measurement）**：
車體座標下、帶有來源與不確定度的速度量測，來源可以是載具模型、測速感測器或上位機。
_Avoid_: 輪速里程計、wheel odom

**局部里程計（Local Odometry）**：
估測器由車體速度量測與航向推算出的局部位姿與速度，不綁定任何一種感測器。
_Avoid_: 輪式里程計

**外部估測器（External Estimator）**：
以上位機送下的位姿（例如 VIO）作為輸出的估測器。
_Avoid_: VIO 直通

**車體運動需求（Body Motion Demand）**：
控制鏈交給載具模型的車體層級需求，例如 twist 或 wrench。
_Avoid_: 車體指令

**致動器（Actuator）**：
把控制輸出轉成運動的元件，例如馬達、舵機、推進器。

**致動器控制（Actuator Control）**：
控制鏈的最後一層，把各致動器的需求轉成實際的輸出訊號。
_Avoid_: 馬達驅動

**控制板（Board）**：
執行韌體的硬體板卡，例如 C50C。
_Avoid_: 主板、target

**硬體版本（Board Revision）**：
同一款控制板的硬體修訂，以原廠韌體回報的名稱稱呼，例如 C50C V1.1。
_Avoid_: V4.1

**板級定義（Board Definition）**：
描述一塊控制板硬體事實的 TOML 檔，包括腳位、周邊、板上裝置與 Flash 配置。
_Avoid_: hwdef、board config

**替代配置（Alt Config）**：
板級定義中，一支腳位在特定 Frame 下的另一種用途。
_Avoid_: AF、複用

**MCU 資料庫（MCU Database）**：
某款 MCU 的腳位、DMA 與 Flash 磁區對照資料，產生器據此驗證板級定義。
_Avoid_: pinmap、chip table

**周邊匯流排（Peripheral Bus）**：
MCU 與板上裝置之間的硬體匯流排，例如 I2C、SPI。
_Avoid_: 匯流排

**遙控器（Remote Control）**：
操作者直接操控載具的輸入裝置，例如 USB 手柄或 SBUS 接收機。
_Avoid_: 手把

**地面站（Ground Station）**：
在載具外、經由 Link 監看或操控載具的工具，例如 QGroundControl。

**上位機（Companion Computer）**：
與 MCU 連線、執行 ROS2 的電腦，例如 Raspberry Pi 5。
_Avoid_: 主機、Pi、host

**開發主機（Development Host）**：
執行建置、測試、SITL 與燒錄工具的電腦，例如開發者的電腦或 CI runner。
_Avoid_: PC、電腦、host

### 運作

**獨立運作（Standalone）**：
沒有上位機時，MCU 僅靠自身估測、控制與遙控輸入維持載具運作的狀態。
_Avoid_: 離線模式

**Setpoint**：
送給 MCU 的控制目標，分為車體速度、姿態/深度、致動器直出三個層級。
_Avoid_: 指令、命令

**模式（Mode）**：
決定哪個來源有控制權、以及它能使用哪些 setpoint 層級的運作狀態。

**Arming**：
致動器是否允許輸出的狀態，分為 armed 與 disarmed。
_Avoid_: 解鎖、上鎖

**安全行為（Failsafe Action）**：
有控制權的來源逾時或失效時，載具自動採取的動作，例如停車、懸停或上浮。
_Avoid_: failsafe 模式

**Setpoint 仲裁（Setpoint Arbitration）**：
MCU 依模式與安全狀態，決定哪個來源的 setpoint 有控制權的機制。
_Avoid_: 指令仲裁、優先權切換

### 日誌

**日誌後端（Log Sink）**：
日誌寫入的目的地，例如 SD 卡、SPI NOR Flash、串流到上位機。
_Avoid_: logger device

**日誌設定檔（Log Profile）**：
一組要記錄的 Topic 和各自的記錄頻率，可以組合使用。
_Avoid_: 日誌等級

**批次訊息（Batch Message）**：
在一則訊息中裝入多個連續樣本的 Topic，用於高頻資料。
_Avoid_: FIFO topic

### 參數

**參數（Parameter）**：
可以在執行時調整、斷電後仍保留的設定值。
_Avoid_: 設定、config

**參數群組（Parameter Group）**：
同一個模組或驅動的參數集合。
_Avoid_: PG

**板級預設（Board Defaults）**：
板級定義提供的參數與 Kconfig 預設值。

**時序預設組（Timing Preset）**：
觸發來源、基準頻率、分頻與取樣方式的一組預設值。
_Avoid_: loop rate

**Frame 預設組（Frame Preset）**：
套用到某一種實際載具配置的一組參數預設值，包含載具類型、Frame 與尺寸。
_Avoid_: airframe、車型參數

### 通訊

**匯流排（Bus）**：
MCU 內部模組之間交換資料的發布/訂閱機制，是韌體的骨幹。
_Avoid_: uORB、message queue

**Topic**：
匯流排上有型別、有名稱的一條資料流，可以有多個實例。

**Work Queue**：
依頻率層級劃分、承載多個工作項目的執行單位。
_Avoid_: 執行緒、thread

**工作項目（Work Item）**：
掛在 Work Queue 上、由計時器或 Topic 發布觸發的模組。
_Avoid_: task、job

**Link**：
MCU 與外部系統之間的一條通訊連線，由 Topic 映射、協議模組與傳輸組成。
_Avoid_: 通道、interface

**協議模組（Protocol）**：
Link 中把 Topic 編碼成特定協議的部分，例如自訂協議、XRCE-DDS、MAVLink。
_Avoid_: driver

**傳輸（Transport）**：
Link 中承載位元組的實體或邏輯通道，例如 UART、CAN、USB。
_Avoid_: 介面、port

**Topic 映射（Topic Map）**：
宣告哪些 Topic 經由某條 Link 對外，以及方向與頻率的設定。

**Schema**：
Topic 與訊息型別的唯一定義來源，MCU、Bridge 與工具的程式碼都由它產生。

**Schema 雜湊（Schema Hash）**：
由 Schema 為每個訊息型別算出的雜湊，用來確認連線雙方的訊息定義一致。

**Bridge**：
上位機端把 Link 轉成 ROS2 topic、service 與 parameter 的節點。
_Avoid_: driver node、agent

### 測試

**測試層（Test Layer）**：
依測試範圍劃分的六個層級，編號 L0–L5。

**Bring-up**：
確認新板子或新接線的接線、對應關係與方向是否正確的測試。
_Avoid_: 開機測試

**測試模式（Test Mode）**：
控制權交給開發主機上測試執行器的一種模式，用來執行 bring-up 案例。

### 軟體分層

**模組層（Module Layer）**：
估測器、控制鏈、載具模型、Link、日誌等功能模組所在的層。
_Avoid_: L3、應用層

**驅動層（Driver Layer）**：
板上與外接裝置的驅動所在的層。
_Avoid_: L2

**介面層（Interface Layer）**：
定義 HAL 契約的層。
_Avoid_: L1

**平台層（Platform Layer）**：
介面層在特定 MCU 或 SITL 上的實作。
_Avoid_: L0、BSP

**能力（Capability）**：
介面層以具體功能提供的平台相關介面，例如 PWM 輸出、正交編碼器。
