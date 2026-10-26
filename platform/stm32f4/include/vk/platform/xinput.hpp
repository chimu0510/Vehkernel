// SPDX-License-Identifier: GPL-3.0-or-later
// The XInput gamepad report (spec 0010): what an Xbox 360 wired controller,
// or a pad such as the Flydigi in its Xbox 360 mode, sends on its interrupt
// IN endpoint. No hardware, so it is tested on the development host.
//
// The input report is 20 bytes: a message type of 0 and a length of 20, the
// buttons as a little-endian 16-bit field already in manual_control_input's
// layout (gamepad_button), the triggers 0 to 255, then the left x, left y,
// right x and right y sticks as little-endian signed 16-bit values, y
// positive up. The interface is class 0xFF, subclass 0x5D, protocol 0x01,
// whatever the vendor and product IDs. Other message types (the LED and
// rumble status) are not input reports.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace vk::platform::xinput {

inline constexpr std::uint8_t interface_class = 0xFF;
inline constexpr std::uint8_t interface_subclass = 0x5D;
inline constexpr std::uint8_t interface_protocol = 0x01;
inline constexpr std::size_t report_size = 20;

// Sets the controller's LED ring to "player 1", which some pads wait for
// before they send input reports.
inline constexpr std::array<std::uint8_t, 3> led_player_1{0x01, 0x03, 0x02};

struct State {
    std::array<float, 4> sticks{};    // left x, left y, right x, right y; -1 to 1, y positive up
    std::array<float, 2> triggers{};  // left, right; 0 to 1
    std::uint16_t buttons = 0;        // bit n: gamepad_button n
};

// The state an input report carries; none for anything else.
inline std::optional<State> parse(std::span<const std::uint8_t> report)
{
    if (report.size() < report_size || report[0] != 0x00 || report[1] < report_size) {
        return std::nullopt;
    }
    const auto u16 = [&report](std::size_t at) {
        return static_cast<std::uint16_t>(report[at] | (report[at + 1] << 8U));
    };
    const auto axis = [&u16](std::size_t at) {
        const auto raw = static_cast<std::int16_t>(u16(at));
        return std::max(static_cast<float>(raw) / 32767.0F, -1.0F);
    };
    State state;
    state.buttons = u16(2);
    state.triggers = {static_cast<float>(report[4]) / 255.0F, static_cast<float>(report[5]) / 255.0F};
    state.sticks = {axis(6), axis(8), axis(10), axis(12)};
    return state;
}

}  // namespace vk::platform::xinput
