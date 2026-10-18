// Platform start-up for STM32F4 (ADR-0009).
#pragma once

namespace vk::platform {

// Configures interrupt priorities, the clock tree from the board definition,
// the microsecond clock and the cycle counter. Call once from the entry point, before anything
// else runs.
void init();

}  // namespace vk::platform
