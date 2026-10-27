# SPDX-License-Identifier: GPL-3.0-or-later
# A body velocity measurement (ADR-0014), in the body frame (x forward,
# y right, z down), with its uncertainty, for the estimators. Each source
# publishes on its own instance: instance 0 is the vehicle model's, from the
# wheel encoders through the pseudo-inverse of its wheel matrix; velocities
# over a Link come later on others.
@queue 4
@instances 2
@rate 200
uint64 timestamp                              # time of publication
uint64 timestamp_sample                       # time of the wheel_encoders sample it was measured from
uint8 source                 @enum velocity_source
float32 vx                   @unit m/s
float32 vy                   @unit m/s        # 0 when the frame cannot move sideways
float32 yaw_rate             @unit rad/s      # positive turning right
float32[3] variance                           # of vx and vy [m^2/s^2] and the yaw rate [rad^2/s^2]
float32 residual             @unit m/s        # RMS wheel rim speed the rigid body does not explain: slip
