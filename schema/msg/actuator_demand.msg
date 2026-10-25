# SPDX-License-Identifier: GPL-3.0-or-later
# The control chain's demand on actuator control (ADR-0013): one wheel speed
# per actuator output, from the vehicle model's allocation of a body
# velocity setpoint, NaN for an output left uncontrolled. Published every
# control cycle; actuator control drives it when its setpoint is newer than
# the last direct-actuator setpoint (ADR-0005).
@rate 200
uint64 timestamp
uint64 timestamp_setpoint                # when the body velocity setpoint it follows was accepted, 0 for none
float32[4] wheel_speed  @unit rad/s     # by output number, output 1 first
