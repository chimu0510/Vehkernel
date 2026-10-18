# A queued, multi-instance topic with a nominal rate.
@queue 4
@instances 2
@rate 200
uint64 timestamp
uint8 instance
float32[3] accel  @unit m/s^2
