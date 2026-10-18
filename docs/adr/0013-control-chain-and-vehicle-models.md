# 一套共用的控制鏈，Frame 差異只放在載具模型

PX4 為每種 rover 各寫一個模組：`rover_differential`、`rover_mecanum`、`rover_ackermann` 各自有一份 Rate、Att、Speed、Pos、Act 控制，等於同一條控制鏈重複了三次。我們採 ArduPilot Rover 的作法：所有 Frame 共用一條控制鏈，Frame 的差異只出現在最後的載具模型（分配）。

控制鏈依序是：

1. setpoint 依 ADR 0003 分層進入：車體速度進入速度控制與角速度控制；姿態或深度進入姿態/深度控制。角速度一律用陀螺儀閉環。
2. 上述控制輸出車體運動需求（twist 或 wrench）。
3. 載具模型把需求分配成各致動器的需求。
4. 致動器控制（輪速 PI 加編碼器）輸出 PWM 與舵機訊號。

致動器直出 setpoint 會跳過第 1～3 步。

**Frame 分兩層**

- **Frame Class**：依分配的數學模型分類，不依載具外觀分類，例如 `WHEELED`、`ACKERMANN`、`THRUSTER`。它決定用哪個載具模型，以及控制鏈要啟用哪些層（深度層另由載具類型決定）。
- **Frame Type**：該 Class 下的具名幾何配置，例如 `DIFF_2WD`、`SKID_4WD`、`MECANUM_4`、`OMNI_3`、`CUSTOM`。
- **Frame 預設組**：實際車款的完整參數，等於載具類型 + Class + Type + 實際尺寸 + 致動器對應 + 調參值。

**載具類型**

參數 `VEHICLE_TYPE` 描述載具運作的環境：`GROUND`、`SURFACE`、`UNDERWATER`，與 Frame 互相獨立。Frame Class 依分配的數學模型分類，分不出水面與水下，所以跟環境有關的事情交給載具類型決定：

- 控制鏈是否啟用深度層。
- 逾時安全行為（ADR 0005）與是否使用磁力計（ADR 0014）的預設值；這些仍是獨立參數，Frame 預設組可以覆寫。
- MAVLink 協議模組回報的 `MAV_TYPE`。

例如雙推進器差速的 USV 是 `SURFACE` + `THRUSTER` + `DIFF_2`，同樣配置的 UUV 只差在 `UNDERWATER`。

**載具模型**

所有載具模型都實作同一個虛擬介面，開機時依 Frame Class 選用。有輪速回授的載具模型（`WHEELED`、`ACKERMANN`）同時發布車體速度量測；積分成里程計是估測器的工作（ADR 0014）。

- **`WHEELED`**：差速、滑移、麥輪、全向輪在速度層級都是線性的，所以統一成一個通用矩陣模型。每個輪子用位置、滾子角度、輪徑描述，組成矩陣 J：
  - 控制時，`輪速 = J·twist`。
  - 車體速度量測用同一個矩陣的虛反矩陣，`twist = J⁺·輪速`，最小平方殘差反映在共變異數中，作為打滑指標。
  - 飽和時整個 twist 等比例縮小，保持行進曲率不變。
- **`ACKERMANN`**：非線性，獨立成一個模型，採自行車模型。
- **`THRUSTER`**：在力與力矩層級分配，`推力 = B⁺·wrench`（同 PX4 的控制分配器）。現階段只定介面。

## Considered Options

- **依外觀分類 Frame Class**（ArduPilot 的 Rover/Boat）：不採用。船的分配方式取決於推進器配置，例如雙推進器差速的 USV，實際上是 `THRUSTER` + `DIFF_2`；水面或水下由載具類型區分。
