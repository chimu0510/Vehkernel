# SPDX-License-Identifier: GPL-3.0-or-later
# Body velocity setpoint (ADR-0003, ADR-0013): the motion of the vehicle as a
# whole, in the body frame (x forward, y right, z down), for the control
# chain. Inbound only: it arrives over a Link and is published once setpoint
# arbitration accepts it, stamped with the time it was accepted (ADR-0005).
# NaN in any field leaves the vehicle uncontrolled.
uint64 timestamp
float32 vx        @unit m/s      # forward
float32 vy        @unit m/s      # right; ignored by frames that cannot move sideways
float32 yaw_rate  @unit rad/s    # about z down, positive turning right
