// SPDX-License-Identifier: GPL-3.0-or-later
// The remote control (spec 0010): the sticks' mapping with its deadzone,
// the buttons' presses and holds, and the input judgements the commander
// shares.
#include <vk/core/manual_control.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/msg/enums.hpp>
#include <vk/msg/manual_control_input.hpp>
#include <vk/remote_control/remote_control.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using vk::core::Command;
using vk::core::Mode;
using vk::core::remote_control;
using vk::msg::GamepadButton;
using vk::remote_control::Input;
using vk::remote_control::RemoteControl;
using vk::remote_control::Settings;

constexpr std::uint64_t cycle_us = 5'000;
const Settings settings{.vx_max = 0.5F, .yaw_max = 1.0F, .deadzone = 0.08F};

std::uint16_t bit(GamepadButton button)
{
    return static_cast<std::uint16_t>(1U << static_cast<unsigned>(button));
}

// Runs the remote control cycle by cycle and collects what it asks.
class RemoteControlTest : public ::testing::Test {
protected:
    // Runs `duration_us` with `input`; the requests made meanwhile.
    std::vector<vk::remote_control::Request> run(const Input& input, std::uint64_t duration_us = cycle_us,
                                                 Mode mode = Mode::Manual,
                                                 vk::core::ControlSource in_control = remote_control)
    {
        std::vector<vk::remote_control::Request> requests;
        for (const std::uint64_t end = now + duration_us; now < end;) {
            now += cycle_us;
            last = rc.step(now, input, mode, in_control, settings);
            for (std::size_t i = 0; i < last.request_count; ++i) {
                requests.push_back(last.requests[i]);
            }
        }
        return requests;
    }

    // The last cycle's setpoint, NaN when there was none.
    [[nodiscard]] vk::remote_control::Twist setpoint() const
    {
        return last.setpoint.value_or(vk::remote_control::Twist{.vx = NAN, .vy = NAN, .yaw_rate = NAN});
    }

    RemoteControl rc;
    vk::remote_control::Result last;
    std::uint64_t now = 1'000'000;
};

TEST(Deadzone, CentresAndRescales)
{
    EXPECT_EQ(vk::core::apply_deadzone(0.05F, 0.08F), 0.0F);
    EXPECT_EQ(vk::core::apply_deadzone(-0.08F, 0.08F), 0.0F);
    EXPECT_NEAR(vk::core::apply_deadzone(0.54F, 0.08F), 0.5F, 1e-6F);
    EXPECT_NEAR(vk::core::apply_deadzone(-1.0F, 0.08F), -1.0F, 1e-6F);
    EXPECT_EQ(vk::core::apply_deadzone(1.5F, 0.08F), 1.0F);
    EXPECT_EQ(vk::core::apply_deadzone(NAN, 0.08F), 0.0F);
    EXPECT_TRUE(vk::core::sticks_centred({0.05F, -0.05F, 0.0F, 0.07F}, 0.08F));
    EXPECT_FALSE(vk::core::sticks_centred({0.0F, 0.0F, 0.0F, 0.2F}, 0.08F));
}

TEST(RcConnected, NeedsAFreshInputThatSaysSo)
{
    vk::msg::ManualControlInput input{};
    input.timestamp = 1'000'000;
    input.connected = true;
    EXPECT_TRUE(vk::core::rc_connected(input, 1'400'000, 500'000));
    EXPECT_FALSE(vk::core::rc_connected(input, 1'600'000, 500'000)) << "silent for longer than RC_TIMEOUT";
    input.connected = false;
    EXPECT_FALSE(vk::core::rc_connected(input, 1'000'000, 500'000));
    EXPECT_FALSE(vk::core::rc_connected(vk::msg::ManualControlInput{}, 0, 500'000)) << "never heard";
}

TEST_F(RemoteControlTest, TheSticksDriveInManualWhenInControl)
{
    // Left stick 0.54 forward, right stick 0.54 right: half of each maximum.
    static_cast<void>(run({.connected = true, .sticks = {0.9F, 0.54F, 0.54F, -0.9F}}));
    ASSERT_TRUE(last.setpoint.has_value());
    EXPECT_NEAR(setpoint().vx, 0.25F, 1e-5F);
    EXPECT_NEAR(setpoint().yaw_rate, 0.5F, 1e-5F) << "right stick right turns right";
    EXPECT_EQ(setpoint().vy, 0.0F) << "left x and right y do nothing";
    // Back and left.
    static_cast<void>(run({.connected = true, .sticks = {0.0F, -1.0F, -1.0F, 0.0F}}));
    EXPECT_NEAR(setpoint().vx, -0.5F, 1e-5F);
    EXPECT_NEAR(setpoint().yaw_rate, -1.0F, 1e-5F);
    // Centred: a zero setpoint, still published.
    static_cast<void>(run({.connected = true, .sticks = {0.0F, 0.03F, -0.05F, 0.0F}}));
    ASSERT_TRUE(last.setpoint.has_value());
    EXPECT_EQ(setpoint().vx, 0.0F);
    EXPECT_EQ(setpoint().yaw_rate, 0.0F);
}

TEST_F(RemoteControlTest, NoSetpointOutsideManualOrWithoutControlOrConnection)
{
    const Input forward{.connected = true, .sticks = {0.0F, 1.0F, 0.0F, 0.0F}};
    static_cast<void>(run(forward, cycle_us, Mode::Test, 0));
    EXPECT_FALSE(last.setpoint.has_value());
    static_cast<void>(run(forward, cycle_us, Mode::Manual, 0));
    EXPECT_FALSE(last.setpoint.has_value());
    static_cast<void>(run({.connected = false, .sticks = {0.0F, 1.0F, 0.0F, 0.0F}}));
    EXPECT_FALSE(last.setpoint.has_value());
}

TEST_F(RemoteControlTest, ATakesOverOncePerPress)
{
    const auto requests = run({.connected = true, .buttons = bit(GamepadButton::A)}, 500'000, Mode::Test, 0);
    ASSERT_EQ(requests.size(), 1U);
    EXPECT_EQ(requests[0].command, Command::Mode);
    EXPECT_EQ(requests[0].argument, static_cast<std::uint8_t>(Mode::Manual));
    // Released and pressed again: once more.
    EXPECT_TRUE(run({.connected = true}).empty());
    EXPECT_EQ(run({.connected = true, .buttons = bit(GamepadButton::A)}).size(), 1U);
}

TEST_F(RemoteControlTest, StartHeldOneSecondArmsAndBackHeldDisarms)
{
    const Input start{.connected = true, .buttons = bit(GamepadButton::Start)};
    // A hold counts from the first cycle that sees the press.
    EXPECT_TRUE(run(start, RemoteControl::hold_us).empty()) << "not yet held long enough";
    const auto armed = run(start, 2 * RemoteControl::hold_us);
    ASSERT_EQ(armed.size(), 1U) << "once per hold";
    EXPECT_EQ(armed[0].command, Command::Arm);
    EXPECT_EQ(armed[0].argument, 1);
    static_cast<void>(run({.connected = true}));
    const auto disarmed =
        run({.connected = true, .buttons = bit(GamepadButton::Back)}, RemoteControl::hold_us + cycle_us);
    ASSERT_EQ(disarmed.size(), 1U);
    EXPECT_EQ(disarmed[0].command, Command::Arm);
    EXPECT_EQ(disarmed[0].argument, 0);
}

TEST_F(RemoteControlTest, BStopsAtOnceAndYHeldReleases)
{
    const auto stop = run({.connected = true, .buttons = bit(GamepadButton::B)});
    ASSERT_EQ(stop.size(), 1U);
    EXPECT_EQ(stop[0].command, Command::EmergencyStop);
    EXPECT_EQ(stop[0].argument, 1);
    // B together with A: the emergency stop goes first.
    static_cast<void>(run({.connected = true}));
    const auto both = run({.connected = true, .buttons = static_cast<std::uint16_t>(bit(GamepadButton::A) |
                                                                                   bit(GamepadButton::B))});
    ASSERT_EQ(both.size(), 2U);
    EXPECT_EQ(both[0].command, Command::EmergencyStop);
    EXPECT_EQ(both[1].command, Command::Mode);
    static_cast<void>(run({.connected = true}));
    const Input y{.connected = true, .buttons = bit(GamepadButton::Y)};
    EXPECT_TRUE(run(y, RemoteControl::hold_us).empty());
    const auto release = run(y, cycle_us);
    ASSERT_EQ(release.size(), 1U);
    EXPECT_EQ(release[0].command, Command::EmergencyStop);
    EXPECT_EQ(release[0].argument, 0);
}

TEST_F(RemoteControlTest, NothingIsAskedWhileDisconnectedAndAHoldStartsOver)
{
    const std::uint16_t start = bit(GamepadButton::Start);
    EXPECT_TRUE(run({.connected = false, .buttons = start}, 2 * RemoteControl::hold_us).empty());
    // Held across a disconnect: the hold starts again when it comes back.
    static_cast<void>(run({.connected = true, .buttons = start}, RemoteControl::hold_us / 2));
    static_cast<void>(run({.connected = false, .buttons = start}));
    EXPECT_TRUE(run({.connected = true, .buttons = start}, RemoteControl::hold_us / 2).empty());
    EXPECT_EQ(run({.connected = true, .buttons = start}, RemoteControl::hold_us / 2 + cycle_us).size(), 1U);
}

}  // namespace
