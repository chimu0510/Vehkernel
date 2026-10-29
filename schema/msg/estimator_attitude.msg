# SPDX-License-Identifier: GPL-3.0-or-later
# The attitude of the vehicle from a secondary estimator (ADR-0014, spec 0012),
# on the instance of its slot (0 Mahony, 1 EKF).
# Fields must stay identical to vehicle_attitude.msg.
@instances 2
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
