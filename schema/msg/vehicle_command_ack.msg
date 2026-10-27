# SPDX-License-Identifier: GPL-3.0-or-later
# The commander's answer to one vehicle_command (ADR-0005), as PX4's
# vehicle_command_ack, with the state a source's reply needs. `result` is the
# command's own result enum: mode_result, arming_result, bringup_start,
# reboot_result or restart_estimators_result, 0 (OK) for an emergency stop request.
@queue 8
uint64 timestamp
uint8 source                             # who asked
uint16 sequence                          # the request's sequence
uint8 command        @enum command
uint8 result                             # 0 when accepted
uint8 mode           @enum mode          # the mode now
bool armed                               # now
bool emergency_stop                      # engaged now
uint32 skipped_checks                    # bits of COM_ARM_CHECK cleared
