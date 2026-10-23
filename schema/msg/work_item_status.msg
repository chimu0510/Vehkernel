# SPDX-License-Identifier: GPL-3.0-or-later
# Timing monitor statistics of one work item (ADR-0015). Each work item
# publishes on its own instance.
@instances 16
@rate 1
uint64 timestamp
uint8 work_item                          # index in the work item table
uint8[16] name                           # work item name, ASCII, zero-padded
uint32 runs                              # runs since boot
uint32 overruns                          # runs that did not finish before the next was due
uint32 run_time_last_us  @unit us
uint32 run_time_max_us   @unit us
uint32 run_time_mean_us  @unit us
uint32 jitter_max_us     @unit us        # largest start delay after the due time
