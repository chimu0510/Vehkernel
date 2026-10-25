# SPDX-License-Identifier: GPL-3.0-or-later
# A request to the commander (ADR-0005), as PX4's vehicle_command: every
# source (each Link, later the remote control and the Bridge) publishes its
# mode, arming, emergency stop, bring-up and reboot requests here, and the
# commander takes them on its own cycle and answers each on
# vehicle_command_ack, matched by source and sequence.
@queue 8
uint64 timestamp
uint8 source                             # who asks: a Link instance, or later the remote control or the Bridge
uint16 sequence                          # the source's own count, echoed in the acknowledgement
uint8 command        @enum command
uint8 argument                           # what the command says it is
