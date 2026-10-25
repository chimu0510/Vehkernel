# SPDX-License-Identifier: GPL-3.0-or-later
# Local odometry (ADR-0014): the vehicle's pose in the local level frame
# (north-east-down, origin and heading zero at boot, as the attitude's) and
# its velocity in the body frame (x forward, y right, z down), dead-reckoned
# from body velocity measurements with the attitude estimator's heading. The
# Bridge publishes it as /odom and TF without computing anything (ADR-0003).
@rate 200
uint64 timestamp                              # time of publication
uint64 timestamp_sample                       # time of the body velocity measurement it was advanced by
uint8 estimator_id                            # which estimator, 1 for dead reckoning
float32[3] position          @unit m          # in the local level frame; down stays 0
float32[4] q                                  # body to level frame, w x y z, from the attitude
float32[3] velocity          @unit m/s        # in the body frame
float32 yaw_rate             @unit rad/s      # the gyro's, bias removed
float32[3] position_variance @unit m^2        # north-north, north-east, east-east
float32 heading_variance     @unit rad^2
float32[3] velocity_variance                  # of vx and vy [m^2/s^2] and the yaw rate [rad^2/s^2]
