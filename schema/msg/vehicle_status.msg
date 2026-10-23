# SPDX-License-Identifier: GPL-3.0-or-later
# The vehicle's mode, arming state and arming checks (ADR-0005), published by
# the commander five times a second and whenever they change.
@rate 5
uint64 timestamp
uint8 mode            @enum mode
bool armed
uint8 source                             # Link instance in control, 255 for none
bool board_verified                      # the IMU answered and the revision-ID pins matched (ADR-0010)
bool emergency_stop                      # engaged
uint8 actuator_power  @enum actuator_power
bool actuator_power_raw                  # the last sample read powered
uint32 skipped_checks                    # bits of COM_ARM_CHECK cleared
uint8 arming_refusal  @enum arming_result # the last arm request's refusal, OK for none
uint8 disarm_reason   @enum disarm_reason # why the vehicle last disarmed, NONE for never
uint32 output_watchdog_trips             # times the output watchdog turned the outputs off
