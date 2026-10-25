# SPDX-License-Identifier: GPL-3.0-or-later
# What the vehicle model made of the control chain's body motion demand
# (ADR-0013): published every control cycle while a body velocity setpoint
# drives, for tests and tools. When a wheel would exceed MOT_MAX_SPD the whole
# demand is scaled down by one factor, and every axis with a demand is
# limited; the controllers stop integrating on a limited axis.
@rate 200
uint64 timestamp
float32 vx           @unit m/s       # the demand before scaling
float32 vy           @unit m/s
float32 yaw_rate     @unit rad/s
float32 scale                        # the factor it was scaled by, 1 when it fits
uint8 limited                        # bits of the limited axes: 1 vx, 2 vy, 4 yaw rate
