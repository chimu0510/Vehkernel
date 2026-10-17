# 內部匯流排為骨幹，ROS2 只是其中一種 Link

MCU 必須能獨立運作，所以韌體內部不能以 ROS2 為骨幹。內部模組之間只透過發布/訂閱匯流排交換資料（同 PX4 的 uORB），對外通訊一律走 Link。Link 拆成三個獨立部分：Topic 映射、協議模組、傳輸，多條 Link 可以同時運作。所有 Topic 由同一份 Schema 定義，MCU、Bridge 與工具的程式碼都由它產生。

第一個協議模組是自訂的輕量協議，分幀用 COBS + CRC16，並帶序號、時間戳、版本協商與時間同步。這樣 MCU 韌體不綁定 ROS2 版本，上位機不在時，Configurator、blackbox 與調參工具也能用同一條鏈路。

## Considered Options

- **micro-ROS（rclc）讓 MCU 成為 ROS2 節點**：不採用。rclc executor 會和控制迴圈共處，需要 Agent，韌體也綁定 ROS2 版本，而且上位機不在時沒有任何工具能用。
- **XRCE-DDS，照 PX4 的作法直接用 Micro-XRCE-DDS-Client，不用 rclc**：列為第二個協議模組，從同一份 Schema 產生，給想直接接標準 Agent 的使用者。
- **zenoh-pico + rmw_zenoh**：MCU 端暫不採用，因為 MCU 端要模擬 rmw_zenoh 的線上格式，序列埠支援也還不成熟。zenoh 放在上位機以上的層級，用於上位機和岸端、地面站、車隊之間的不穩定或多跳鏈路。
- **MAVLink**：列為選配協議模組，讓獨立運作的載具可以直接接數傳電台與 QGroundControl。只借用它的通訊生態，不承擔導航職責。

## Consequences

- 從任何 Link 進來的 setpoint 都必須先經過指令仲裁，不可直接驅動致動器。
