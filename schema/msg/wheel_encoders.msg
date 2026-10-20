# Each actuator output's wheel, from its quadrature encoder (ADR-0013), every
# control cycle. Outputs the board does not have are not valid.
@rate 200
uint64 timestamp
float32[4] angle  @unit rad             # accumulated since boot
float32[4] speed  @unit rad/s
uint8 valid                             # bit n: output n + 1 has an encoder
