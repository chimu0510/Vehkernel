// Platform start-up for SITL (ADR-0009, ADR-0018).
#pragma once

#include <cstdint>

namespace vk::platform {

// Starts the microsecond clock. Call once from the entry point, before
// anything else runs.
void init();

// Stops the scheduler after the given run time, so a SITL run ends on its own.
void stop_after(std::uint64_t duration_us);

}  // namespace vk::platform
