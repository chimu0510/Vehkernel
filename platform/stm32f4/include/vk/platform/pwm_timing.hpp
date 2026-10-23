// SPDX-License-Identifier: GPL-3.0-or-later
// PWM timer settings for STM32F4 motor outputs (spec 0006).
//
// Free of hardware access, so it is unit-tested on the development host.
#pragma once

#include <cstdint>
#include <optional>

namespace vk::platform {

// Prescaler and auto-reload register values: the timer counts
// `period + 1` ticks of timer_hz / (prescaler + 1) per PWM cycle.
struct PwmTiming {
    std::uint32_t prescaler;
    std::uint32_t period;

    friend constexpr bool operator==(const PwmTiming&, const PwmTiming&) = default;
};

// The smallest prescaler that fits a 16-bit period at `hz`, for the finest
// duty steps; none when that gives fewer than `min_steps` steps or misses
// `hz` by more than 1 %.
constexpr std::optional<PwmTiming> pwm_timing(std::uint32_t timer_hz, std::uint32_t hz,
                                              std::uint32_t min_steps = 1000)
{
    if (hz == 0) {
        return std::nullopt;
    }
    for (std::uint32_t prescaler = 0; prescaler <= 0xFFFF; ++prescaler) {
        const std::uint32_t ticks = timer_hz / ((prescaler + 1) * hz);
        if (ticks > 0x10000) {
            continue;
        }
        if (ticks < min_steps) {
            return std::nullopt;
        }
        const std::uint32_t actual = timer_hz / ((prescaler + 1) * ticks);
        const std::uint32_t error = actual > hz ? actual - hz : hz - actual;
        if (error * 100 > hz) {
            return std::nullopt;
        }
        return PwmTiming{.prescaler = prescaler, .period = ticks - 1};
    }
    return std::nullopt;
}

// An advanced timer's (TIM1, TIM8) break and dead-time register: main
// outputs enabled (MOE), and OSSI, so that whenever MOE is cleared, as when
// a debugger halts the core with DBG_TIMx_STOP set, the outputs are driven
// to their idle level instead of floating.
inline constexpr std::uint32_t advanced_timer_moe = 1U << 15U;
inline constexpr std::uint32_t advanced_timer_ossi = 1U << 10U;
inline constexpr std::uint32_t advanced_timer_bdtr = advanced_timer_moe | advanced_timer_ossi;

// Its CR2 output idle bits, OIS1 to OIS4: all cleared, so the idle level is low.
inline constexpr std::uint32_t advanced_timer_ois_mask = (1U << 8U) | (1U << 10U) | (1U << 12U) | (1U << 14U);
inline constexpr std::uint32_t advanced_timer_ois = 0;

}  // namespace vk::platform
