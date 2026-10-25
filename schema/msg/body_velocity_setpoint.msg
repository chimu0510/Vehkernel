# SPDX-License-Identifier: GPL-3.0-or-later
# Body velocity setpoint (ADR-0003, ADR-0013): the motion of the vehicle as a
# whole, in the body frame (x forward, y right, z down), for the control
# chain. It arrives over a Link, stamped with the time it arrived and tagged
# with the Link as its source; its consumers drop it unless the mode accepts
# it from that source (setpoint arbitration, ADR-0005). NaN in any field
# leaves the vehicle uncontrolled.
uint64 timestamp
uint8 source                    # who sent it: a Link instance, or later the remote control
float32 vx        @unit m/s      # forward
float32 vy        @unit m/s      # right; ignored by frames that cannot move sideways
float32 yaw_rate  @unit rad/s    # about z down, positive turning right
