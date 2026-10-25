# SPDX-License-Identifier: GPL-3.0-or-later
# The true pose and velocity of SITL's ground vehicle, published by SITL only
# (ADR-0018), to check the control chain and the odometry against: the local
# level frame (north-east-down, origin and heading zero at boot) and the body
# frame (x forward, y right, z down).
@rate 200
uint64 timestamp                          # time of the IMU sample it belongs to
float32[3] position          @unit m      # in the local level frame
float32[4] q                              # body to level frame, w x y z
float32[3] velocity          @unit m/s    # in the body frame
float32[3] angular_velocity  @unit rad/s  # in the body frame
