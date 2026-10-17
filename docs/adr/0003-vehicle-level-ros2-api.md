# 對 ROS2 提供載具級 topic API，ros2_control 只是選配 adapter

運動學與控制分配在 MCU 上，而且 MCU 能獨立閉環，所以 MCU 並不是 ros2_control 假設的那種關節級「笨」硬體。這和 PX4 的處境相同，PX4 因此也用 topic 介面。Bridge 對 ROS2 提供載具級 API：

- 輸出：odom、imu、電池、健康與模式狀態。TF（odom → base_link）由 Bridge 依照 MCU 的時間戳發布，MCU 不需要懂 TF。
- 輸入：三個層級的 setpoint，分別是車體速度（Twist）、姿態/深度、致動器直出。
- arming 與切換模式用 service；MCU 參數映射成 ROS2 parameters。

Nav2 只需要 `/cmd_vel`、`/odom` 與 TF，用這套 API 就能直接接。

## Considered Options

- **以 ros2_control 為主介面**：不採用。`diff_drive_controller` 會在上位機再做一次逆運動學，odom 就有兩個來源；水面與水下的推進器分配也沒有標準 controller。ros2_control 保留為選配 adapter，此時 MCU 切到致動器直出模式，只做安全守門，運動學交給上位機，給需要自訂 controller 的整合場景使用。
