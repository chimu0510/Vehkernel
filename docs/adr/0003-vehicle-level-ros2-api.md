# Vehicle-level topic API for ROS2; ros2_control is an optional adapter

Control allocation (the vehicle model) runs on the MCU and the MCU closes its own loops, so it is not the joint-level "dumb" hardware ros2_control assumes. PX4 is in the same position and uses a topic interface for the same reason. The Bridge exposes a vehicle-level API to ROS2:

- Outputs: odometry, IMU, battery, health and mode status. The Bridge publishes the TF (odom → base_link) from MCU timestamps, so the MCU never deals with TF. When the external estimator is enabled, either the Bridge or the VIO node publishes it, not both (ADR-0014).
- Inputs: setpoints at three levels: body velocity (Twist), attitude/depth and direct actuator.
- State measurement inputs: the companion computer may send body velocity measurements or external poses, for example from VIO (ADR-0014). These are measurements, not setpoints, so they bypass setpoint arbitration.
- Services for arming and mode changes; MCU parameters are mapped to ROS2 parameters.

Nav2 needs only `/cmd_vel`, `/odom` and TF, so it connects to this API directly.

## Considered Options

- **ros2_control as the primary interface**: rejected. `diff_drive_controller` would redo inverse kinematics on the companion computer, giving two sources of odometry, and there is no standard controller for surface or underwater thruster allocation. ros2_control remains an optional adapter for integrations that need a custom controller: the companion computer then sends direct-actuator setpoints, the MCU acts only as safety gatekeeper, and allocation moves to the companion computer.
