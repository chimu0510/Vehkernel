# SPDX-License-Identifier: GPL-3.0-or-later
# The bring-up case actuator control runs (ADR-0018), published whenever its
# state changes. Results are by output number, output 1 first.
uint64 timestamp
uint16 run                              # counts every case started since boot
uint8 bringup_case  @enum bringup_case
uint8 source                            # Link instance that asked for it
uint8 state  @enum bringup_state
uint8 outputs                           # how many outputs the case covered
uint8[4] result  @enum bringup_result
uint8[4] encoder                        # the encoder that moved most, 1 to 4, 0 for none
int8[4] sign                            # the sign of that encoder's count change
