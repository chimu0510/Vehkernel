# Direct-actuator setpoint (ADR-0003, ADR-0013): one wheel speed per actuator
# output, NaN for an output left uncontrolled. Inbound only: it arrives over a
# Link and is published once setpoint arbitration accepts it, stamped with the
# time it was accepted (ADR-0005).
uint64 timestamp
float32[4] wheel_speed  @unit rad/s     # by output number, output 1 first
