// SPDX-License-Identifier: GPL-3.0-or-later
// The XInput gamepad report (spec 0010): every field of an input report, the
// stick extremes, and what is not an input report.
#include <vk/msg/enums.hpp>
#include <vk/platform/xinput.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace {

namespace xinput = vk::platform::xinput;
using vk::msg::GamepadButton;

// An input report with the given buttons, triggers and sticks.
std::array<std::uint8_t, 20> report(std::uint16_t buttons, std::uint8_t lt, std::uint8_t rt,
                                    std::array<std::int16_t, 4> sticks)
{
    std::array<std::uint8_t, 20> r{};
    r[0] = 0x00;
    r[1] = 0x14;
    r[2] = static_cast<std::uint8_t>(buttons & 0xFFU);
    r[3] = static_cast<std::uint8_t>(buttons >> 8U);
    r[4] = lt;
    r[5] = rt;
    for (std::size_t i = 0; i < sticks.size(); ++i) {
        const auto raw = static_cast<std::uint16_t>(sticks[i]);
        r[6 + 2 * i] = static_cast<std::uint8_t>(raw & 0xFFU);
        r[7 + 2 * i] = static_cast<std::uint8_t>(raw >> 8U);
    }
    return r;
}

std::uint16_t bit(GamepadButton button)
{
    return static_cast<std::uint16_t>(1U << static_cast<unsigned>(button));
}

TEST(Xinput, CentredAndReleased)
{
    const auto state = xinput::parse(report(0, 0, 0, {0, 0, 0, 0}));
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state.value_or(xinput::State{}).buttons, 0);
    for (const float stick : state.value_or(xinput::State{}).sticks) {
        EXPECT_EQ(stick, 0.0F);
    }
}

TEST(Xinput, SticksAtTheirExtremes)
{
    // Left stick full right and full up; right stick full left and full down.
    const auto state = xinput::parse(report(0, 0, 0, {32767, 32767, -32768, -32767})).value_or(xinput::State{});
    EXPECT_FLOAT_EQ(state.sticks[0], 1.0F);
    EXPECT_FLOAT_EQ(state.sticks[1], 1.0F) << "y positive up: forward";
    EXPECT_FLOAT_EQ(state.sticks[2], -1.0F) << "-32768 clamps to -1";
    EXPECT_FLOAT_EQ(state.sticks[3], -1.0F);
    const auto half = xinput::parse(report(0, 0, 0, {16384, -16384, 0, 0})).value_or(xinput::State{});
    EXPECT_NEAR(half.sticks[0], 0.5F, 1e-4F);
    EXPECT_NEAR(half.sticks[1], -0.5F, 1e-4F);
}

TEST(Xinput, TriggersAndEveryButton)
{
    const auto state = xinput::parse(report(0, 255, 128, {0, 0, 0, 0})).value_or(xinput::State{});
    EXPECT_FLOAT_EQ(state.triggers[0], 1.0F);
    EXPECT_NEAR(state.triggers[1], 128.0F / 255.0F, 1e-6F);
    // The report's button field is already in gamepad_button's layout.
    for (const auto& entry : vk::msg::EnumValues<GamepadButton>::values) {
        const auto pressed = xinput::parse(report(bit(entry.value), 0, 0, {0, 0, 0, 0})).value_or(xinput::State{});
        EXPECT_EQ(pressed.buttons, bit(entry.value)) << entry.name;
    }
    // START is byte 2 bit 4; A is byte 3 bit 4.
    const auto raw = report(0, 0, 0, {0, 0, 0, 0});
    auto start_a = raw;
    start_a[2] = 0x10;
    start_a[3] = 0x10;
    EXPECT_EQ(xinput::parse(start_a).value_or(xinput::State{}).buttons,
              bit(GamepadButton::Start) | bit(GamepadButton::A));
}

TEST(Xinput, OnlyInputReportsAreRead)
{
    auto led_status = report(0, 0, 0, {0, 0, 0, 0});
    led_status[0] = 0x01;  // LED status
    led_status[1] = 0x03;
    EXPECT_FALSE(xinput::parse(led_status).has_value());
    const auto full = report(0, 0, 0, {0, 0, 0, 0});
    EXPECT_FALSE(xinput::parse(std::span(full).first(19)).has_value()) << "too short";
}

}  // namespace
