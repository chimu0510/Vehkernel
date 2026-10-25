# SPDX-License-Identifier: GPL-3.0-or-later
# Direct-actuator setpoint (ADR-0003, ADR-0013): one wheel speed per actuator
# output, NaN for an output left uncontrolled. Inbound only: it arrives over a
# Link, stamped with the time it arrived and tagged with the Link as its
# source; its consumers drop it unless the mode accepts it from that source
# (setpoint arbitration, ADR-0005).
uint64 timestamp
uint8 source                            # who sent it: a Link instance, or later the remote control
float32[4] wheel_speed  @unit rad/s     # by output number, output 1 first
