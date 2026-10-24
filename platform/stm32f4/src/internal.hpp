// SPDX-License-Identifier: GPL-3.0-or-later
// Start-up steps shared between the STM32F4 platform sources.
#pragma once

namespace vk::platform::detail {

void configure_clock_tree();
void start_microsecond_clock();

// Allows writes to the RTC backup registers, which survive a reset: the
// bootloader request (reset.cpp) and the fault record (fault.cpp). Writes
// registers only, so the fault stop may call it.
void unlock_backup_registers();

// Reads and clears the reset flags and the fault record; once, at boot.
void read_reset_report();

// The fault stop's first step (ADR-0005): every advanced timer's main output
// off, then the stop given to the output watchdog. Writes registers only.
void force_outputs_off();

}  // namespace vk::platform::detail
