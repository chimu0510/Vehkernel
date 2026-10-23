// SPDX-License-Identifier: GPL-3.0-or-later
// Extending a 16-bit hardware timer to the 64-bit microsecond clock.
//
// The timer counts microseconds and its overflow interrupt counts wraps. A
// reading taken with interrupts masked may find a wrap that has happened but
// is not yet counted; it belongs to the reading only if the counter value was
// taken after the wrap, i.e. is in the lower half of its range. Kept free of
// hardware access so it is tested on the development host.
#pragma once

#include <cstdint>

namespace vk::platform {

[[nodiscard]] constexpr std::uint64_t extend_counter(std::uint64_t overflows, std::uint16_t counter,
                                                     bool overflow_pending) noexcept
{
    constexpr std::uint16_t half_range = 0x8000;
    if (overflow_pending && counter < half_range) {
        ++overflows;
    }
    return (overflows << 16U) | counter;
}

}  // namespace vk::platform
