# SPDX-License-Identifier: GPL-3.0-or-later
# The attitude of the vehicle from the primary estimator (ADR-0014): body
# frame x forward, y right, z down, relative to the local level frame
# (north-east-down, yaw relative to north when the magnetometer is fused,
# and to the start otherwise).
uint64 timestamp                          # time of publication
uint64 timestamp_sample                   # time of the IMU sample it was computed from
uint8 estimator_id                        # which estimator, 1 for Mahony, 2 for EKF
float32[4] q                              # body to level frame, w x y z
float32[3] angular_velocity  @unit rad/s  # body rate, gyro bias removed
float32[3] gyro_bias         @unit rad/s  # current gyro bias estimate
float32[3] variance          @unit rad^2  # of roll, pitch and yaw
uint32 accel_rejected                     # samples whose accelerometer correction was skipped
uint32 mag_rejected                       # samples whose magnetometer correction was skipped
uint32 mag_fused                          # samples whose magnetometer correction was fused
uint32 zero_rate_fused                    # zero rate updates fused while at rest
bool mag_missing                          # heading fusion on but no magnetometer during alignment
