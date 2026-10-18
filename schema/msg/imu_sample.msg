# One IMU sample, rotated into the body frame and in SI units (ADR-0014).
# Queued so a batch of samples read at once is never overwritten (ADR-0015);
# the nominal rate comes from the board's timing preset, not the schema.
@queue 4
@instances 2
uint64 timestamp                 # time the sample was taken
uint8 device_instance            # which IMU, 0 for the first
float32[3] accel  @unit m/s^2    # specific force, x forward, y right, z down
float32[3] gyro   @unit rad/s    # angular rate about x, y, z
float32 temperature  @unit degC  # sensor die temperature
