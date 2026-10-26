// SPDX-License-Identifier: GPL-3.0-or-later
// How the remote control's input is judged (spec 0010), shared by the remote
// control, which drives from it, and the commander, which arms on it and
// fails safe without it.
#pragma once

#include <vk/msg/manual_control_input.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace vk::core {

// Whether `input` counts as connected at `now_us`: it says so and is no older
// than `timeout_us` (RC_TIMEOUT).
constexpr bool rc_connected(const msg::ManualControlInput& input, std::uint64_t now_us, std::uint64_t timeout_us)
{
    return input.connected && input.timestamp != 0 && now_us >= input.timestamp &&
           now_us - input.timestamp <= timeout_us;
}

// `value` after the deadzone (MAN_DEADZONE): 0 inside it, rescaled to reach
// ±1 at full travel.
inline float apply_deadzone(float value, float deadzone)
{
    if (!std::isfinite(value)) {
        return 0.0F;
    }
    const float magnitude = std::fabs(value);
    if (magnitude <= deadzone || deadzone >= 1.0F) {
        return 0.0F;
    }
    return std::copysign(std::min((magnitude - deadzone) / (1.0F - deadzone), 1.0F), value);
}

// Whether every stick is inside the deadzone.
inline bool sticks_centred(const std::array<float, 4>& sticks, float deadzone)
{
    return std::ranges::all_of(sticks, [deadzone](float s) { return apply_deadzone(s, deadzone) == 0.0F; });
}

}  // namespace vk::core
