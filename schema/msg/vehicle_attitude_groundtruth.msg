# The true attitude of the simulated vehicle, published by SITL only
# (ADR-0018), to check estimators against: same frames as vehicle_attitude.
uint64 timestamp                          # time of the IMU sample it belongs to
float32[4] q                              # body to level frame, w x y z
float32[3] angular_velocity  @unit rad/s  # true body rate
