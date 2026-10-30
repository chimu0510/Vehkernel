// SPDX-License-Identifier: GPL-3.0-or-later
// Microsecond clock for SITL: host monotonic time since the first reading.
#include <vk/hal/clock.hpp>

#include <chrono>
#include <cstdint>

namespace {

std::chrono::steady_clock::time_point boot_time()
{
    static const auto boot = std::chrono::steady_clock::now();
    return boot;
}

}  // namespace

std::uint64_t vk::hal::SystemClock::now_us() noexcept
{
    // Boot first: the operands of '-' have no evaluation order, and on the first
    // call a later boot than now would make elapsed negative and wrap.
    const auto boot = boot_time();
    const auto elapsed = std::chrono::steady_clock::now() - boot;
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}
