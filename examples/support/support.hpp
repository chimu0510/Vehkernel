// SPDX-License-Identifier: GPL-3.0-or-later
// Support code for board examples (spec 0013, ADR-0025).
#pragma once

#include <vk/core/scheduler.hpp>
#include <vk/platform/platform.hpp>

#include <cstdint>
#include <string_view>

namespace example {

// Brings up the platform: configures fault handlers, interrupt priorities,
// the clock tree from the board definition, the microsecond clock and the
// cycle counter (ADR-0009).
inline void init()
{
    vk::platform::init();
}

// Starts the FreeRTOS scheduler (ADR-0004). Never returns.
[[noreturn]] inline void start_scheduler()
{
    vk::core::start_scheduler();
}

// Emits a key=value line in SITL for L3 testing (spec 0013); does nothing on
// the board.
template <typename T>
inline void report(std::string_view key, const T& value)
{
    static_cast<void>(key);
    static_cast<void>(value);
}

}  // namespace example
