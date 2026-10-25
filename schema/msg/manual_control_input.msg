# SPDX-License-Identifier: GPL-3.0-or-later
# The remote control's input (spec 0010), as PX4's manual_control_setpoint:
# the gamepad's sticks, triggers and buttons, from whatever reads it. It is
# published at least 20 times a second while the gamepad is connected, its
# latest state repeated when the gamepad sends nothing new (an XInput pad
# reports only changes), so silence means the remote control is lost; and
# once with connected false when it goes. It knows nothing of modes.
@rate 50
uint64 timestamp
uint8 source        @enum manual_control_source
bool connected
float32[4] sticks                        # left x, left y, right x, right y; -1 to 1, x right, y forward
float32[2] triggers                      # left, right; 0 to 1
uint16 buttons                           # bit n: gamepad_button n pressed
