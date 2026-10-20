# What actuator control applied to each output (ADR-0013), every control
# cycle: the duty, its sign the direction, and the speed target after
# clamping, NaN for an output left uncontrolled.
@rate 200
uint64 timestamp
float32[4] output                       # duty from -1 to 1
float32[4] target  @unit rad/s
