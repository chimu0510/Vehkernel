// Platform start-up for STM32F4 (ADR-0009).
#pragma once

#include <cstdint>

namespace vk::platform {

// Configures interrupt priorities, the clock tree from the board definition,
// the microsecond clock and the cycle counter. Call once from the entry point, before anything
// else runs.
void init();

// Starts the output watchdog (ADR-0005) for actuator control running every
// `period_us`: a TIM6 interrupt above FreeRTOS that calls `stop` when the
// outputs have not been refreshed for three periods.
void start_output_watchdog(std::uint32_t period_us, void (*stop)());
std::uint32_t output_watchdog_trips();

}  // namespace vk::platform
