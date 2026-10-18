# The MCU does real-time estimation and low-level control, not navigation

The firmware targets ground, surface and underwater vehicles first; aircraft and other fast-responding vehicles come later. The MCU must keep the vehicle running without a companion computer, so it keeps basic state estimation, attitude and low-level control, the vehicle model (control allocation) and the safety mechanisms. Global localization, mapping and navigation belong to ROS2 applications above it. This is a deliberate departure from ArduPilot and PX4, which run autonomous navigation on the MCU: here the MCU is the companion computer's real-time coprocessor and safety gatekeeper.

## Consequences

- No GPS-fused global EKF, waypoint missions or path planning on the MCU, even once USVs are supported.
- Although the first vehicles have relaxed timing needs, the scheduler is still split into rate tiers, so that extending to aircraft later does not mean rewriting the control path.
