// SPDX-License-Identifier: GPL-3.0-or-later
// Monotonic microsecond clock contract (ADR-0008, ADR-0009).
//
// Every topic message carries a 64-bit microsecond timestamp taken from this
// clock. One build has exactly one platform, so the clock is a concrete class
// whose definition the platform layer links in; the concept states the
// contract each platform's definition must meet.
#pragma once

#include <concepts>
#include <cstdint>

namespace vk::hal {

// A clock counting microseconds since boot, never going backwards and never
// wrapping in the lifetime of a vehicle.
template <class T>
concept MicrosecondClock = requires {
    { T::now_us() } noexcept -> std::same_as<std::uint64_t>;
};

// The build's clock. Defined by the platform layer.
struct SystemClock {
    static std::uint64_t now_us() noexcept;
};

static_assert(MicrosecondClock<SystemClock>);

}  // namespace vk::hal
