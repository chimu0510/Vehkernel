// SPDX-License-Identifier: GPL-3.0-or-later
// Platform start-up for STM32F4 (ADR-0009).
#pragma once

#include <vk/core/fault.hpp>
#include <vk/hal/reset.hpp>

#include <cstdint>

namespace vk::platform {

// Reads the reset cause, then configures the fault handlers, interrupt
// priorities, the clock tree from the board definition, the microsecond clock
// and the cycle counter. Call once from the entry point, before anything else
// runs.
void init();

// What caused the last reset, read by init() (ADR-0005).
core::ResetReport reset_report();

// Starts the hardware watchdog (the IWDG, ADR-0005), refreshed from the
// FreeRTOS idle hook: call it just before the scheduler starts. Without it a
// fault still starts the IWDG to reset the board.
void start_hardware_watchdog();
void refresh_hardware_watchdog();

// Starts the output watchdog (ADR-0005) for actuator control running every
// `period_us`: a TIM6 interrupt above FreeRTOS that calls `stop` when the
// outputs have not been refreshed for three periods. The fault stop calls
// `stop` too.
void start_output_watchdog(std::uint32_t period_us, void (*stop)());
std::uint32_t output_watchdog_trips();

// Restarts the board into the firmware or the bootloader (ADR-0023); a
// hal::Reset.
[[noreturn]] void reset(hal::BootTarget target);

}  // namespace vk::platform
