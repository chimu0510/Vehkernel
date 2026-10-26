// SPDX-License-Identifier: GPL-3.0-or-later
// The control chain (ADR-0013): a body velocity setpoint through angular
// rate control on the gyro and the vehicle model to a wheel speed target per
// output, with the integrator frozen on a limited axis (#84) and reset when
// the loops stop closing or the vehicle is at rest, and the rate of change
// of vx and vy limited (#133).
#include <vk/control/control_chain.hpp>
#include <vk/msg/enums.hpp>
#include <vk/vehicle_model/wheeled_model.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace {

using vk::control::ChainResult;
using vk::control::ControlChain;
using vk::control::RateSettings;
using vk::control::Twist;
using vk::control::VelocityLimits;
using vk::control::TwistAxis;
using vk::msg::FrameType;
using vk::vehicle_model::max_outputs;
using vk::vehicle_model::WheeledModel;

using Targets = std::array<float, max_outputs>;

constexpr float dt = 0.005F;
constexpr float max_speed = 30.0F;
const WheeledModel diff_2wd({.type = FrameType::Diff2wd, .radius = 0.05F, .track = 0.2F, .outputs = {1, 2, 3, 4}});
// Feedforward only, for checks of the allocation itself.
constexpr RateSettings open_loop{.p = 0.0F, .i = 0.0F, .ff = 1.0F};
constexpr RateSettings pi{.p = 0.5F, .i = 2.0F, .ff = 1.0F};

ChainResult run(ControlChain& chain, const Twist& setpoint, std::optional<float> measured, bool closed,
                const RateSettings& settings, int cycles = 1, float limit = max_speed)
{
    Targets targets{};
    ChainResult result{};
    for (int i = 0; i < cycles; ++i) {
        result = chain.step(dt, setpoint, measured, closed, settings, limit, targets);
    }
    return result;
}

TEST(ControlChain, ABodyVelocityBecomesWheelSpeeds)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    const auto result = chain.step(dt, {.vx = 0.5F, .yaw_rate = 1.0F}, 1.0F, true, open_loop, max_speed, targets);
    EXPECT_FLOAT_EQ(result.allocation.scale, 1.0F);
    EXPECT_FLOAT_EQ(targets[0], 12.0F);
    EXPECT_FLOAT_EQ(targets[1], 8.0F);
    EXPECT_TRUE(std::isnan(targets[2]));
    EXPECT_TRUE(std::isnan(targets[3]));
}

TEST(ControlChain, ATwistBeyondReachIsScaledAndReported)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    const auto result = chain.step(dt, {.vx = 1.0F, .yaw_rate = 2.0F}, 2.0F, true, open_loop, 12.0F, targets);
    EXPECT_FLOAT_EQ(result.allocation.scale, 0.5F);
    EXPECT_EQ(result.allocation.limited,
              static_cast<std::uint8_t>(TwistAxis::Vx) | static_cast<std::uint8_t>(TwistAxis::YawRate));
    EXPECT_FLOAT_EQ(targets[0], 12.0F);
    EXPECT_FLOAT_EQ(targets[1], 8.0F);
}

TEST(ControlChain, NoSetpointOrNoModelDrivesNothing)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    static_cast<void>(chain.step(dt, {.vx = NAN, .vy = NAN, .yaw_rate = NAN}, 0.0F, true, pi, max_speed, targets));
    for (const float target : targets) {
        EXPECT_TRUE(std::isnan(target));
    }
    ControlChain none(nullptr);
    EXPECT_FALSE(none.has_model());
    static_cast<void>(none.step(dt, {.vx = 0.5F}, 0.0F, true, pi, max_speed, targets));
    for (const float target : targets) {
        EXPECT_TRUE(std::isnan(target));
    }
}

TEST(ControlChain, TheYawRateLoopIsFeedforwardPlusPi)
{
    ControlChain chain(&diff_2wd);
    // On target: the demand is the feedforward alone.
    EXPECT_FLOAT_EQ(run(chain, {.yaw_rate = 1.0F}, 1.0F, true, pi).demand.yaw_rate, 1.0F);
    // 0.25 rad/s short, as when the wheels slip: P adds 0.125 and the
    // integral grows by 2 x 0.25 x 0.005 each cycle.
    const auto first = run(chain, {.yaw_rate = 1.0F}, 0.75F, true, pi);
    EXPECT_NEAR(first.demand.yaw_rate, 1.0F + 0.125F + 0.0025F, 1e-6F);
    EXPECT_NEAR(chain.yaw_integral(), 0.0025F, 1e-7F);
    const auto later = run(chain, {.yaw_rate = 1.0F}, 0.75F, true, pi, 99);
    EXPECT_NEAR(chain.yaw_integral(), 0.25F, 1e-4F);
    EXPECT_NEAR(later.demand.yaw_rate, 1.0F + 0.125F + 0.25F, 1e-4F);
    // vx passes through as feedforward.
    EXPECT_FLOAT_EQ(run(chain, {.vx = 0.4F, .yaw_rate = 1.0F}, 1.0F, true, pi).demand.vx, 0.4F);
}

TEST(ControlChain, TheIntegralFreezesOnALimitedYawAxis)
{
    ControlChain chain(&diff_2wd);
    // 4 rad/s of yaw needs 8 rad/s per wheel; a 6 rad/s limit holds the axis.
    static_cast<void>(run(chain, {.yaw_rate = 4.0F}, 3.0F, true, pi, 1, 6.0F));
    const float frozen = chain.yaw_integral();
    const auto held = run(chain, {.yaw_rate = 4.0F}, 3.0F, true, pi, 100, 6.0F);
    EXPECT_NE(held.allocation.limited & static_cast<std::uint8_t>(TwistAxis::YawRate), 0);
    EXPECT_FLOAT_EQ(chain.yaw_integral(), frozen);
    // Within reach again, it integrates.
    static_cast<void>(run(chain, {.yaw_rate = 1.0F}, 0.5F, true, pi, 2));
    EXPECT_GT(chain.yaw_integral(), frozen);
}

TEST(ControlChain, ASaturatedLoopKeepsTheCurvature)
{
    ControlChain chain(&diff_2wd);
    // 0.4 m/s at 2 rad/s needs 12 and 4 rad/s; a 10 rad/s limit scales it by 5/6.
    const Twist setpoint{.vx = 0.4F, .yaw_rate = 2.0F};
    // The vehicle turns at what the scaled demand gives, 5/6 of 2 rad/s: no
    // error remains, so the demand keeps the setpoint's ratio of vx to yaw
    // rate. Only the first cycle, before any limit was reported, integrated
    // (2 x 2 x 0.005), and the integral then froze.
    const auto result = run(chain, setpoint, 2.0F * 10.0F / 12.0F, true, pi, 50, 10.0F);
    EXPECT_NEAR(chain.yaw_integral(), 0.0033F, 1e-3F);
    EXPECT_NEAR(result.allocation.scale, 10.0F / 12.0F, 2e-3F);
    EXPECT_NEAR(result.demand.vx / result.demand.yaw_rate, setpoint.vx / setpoint.yaw_rate, 1e-3F);
}

TEST(ControlChain, TheIntegralResetsWhenTheLoopsStopClosing)
{
    ControlChain chain(&diff_2wd);
    static_cast<void>(run(chain, {.yaw_rate = 1.0F}, 0.5F, true, pi, 100));
    ASSERT_GT(chain.yaw_integral(), 0.1F);
    // Disarmed or the emergency stop: open loop, the integral cleared.
    const auto open = run(chain, {.yaw_rate = 1.0F}, 0.5F, false, pi);
    EXPECT_FLOAT_EQ(chain.yaw_integral(), 0.0F);
    EXPECT_FLOAT_EQ(open.demand.yaw_rate, 1.0F);
    // No setpoint clears it too.
    static_cast<void>(run(chain, {.yaw_rate = 1.0F}, 0.5F, true, pi, 100));
    static_cast<void>(run(chain, {.vx = NAN, .vy = NAN, .yaw_rate = NAN}, 0.5F, true, pi));
    EXPECT_FLOAT_EQ(chain.yaw_integral(), 0.0F);
}

TEST(ControlChain, WithoutAMeasurementTheIntegralHolds)
{
    ControlChain chain(&diff_2wd);
    static_cast<void>(run(chain, {.yaw_rate = 1.0F}, 0.5F, true, pi, 100));
    const float held = chain.yaw_integral();
    const auto result = run(chain, {.yaw_rate = 1.0F}, std::nullopt, true, pi, 10);
    EXPECT_FLOAT_EQ(chain.yaw_integral(), held);
    EXPECT_FLOAT_EQ(result.demand.yaw_rate, 1.0F + held);
}

TEST(ControlChain, TheSpeedChangesNoFasterThanTheLimits)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    const VelocityLimits limits{.acc_max = 1.0F, .dec_max = 2.0F};
    auto step = [&](const Twist& setpoint) {
        return chain.step(dt, setpoint, 0.0F, true, open_loop, max_speed, targets, limits).demand;
    };
    // 0.5 m/s at 1 m/s²: 0.005 m/s a cycle, there after 100 cycles.
    EXPECT_NEAR(step({.vx = 0.5F}).vx, 0.005F, 1e-6F);
    for (int i = 0; i < 98; ++i) {
        static_cast<void>(step({.vx = 0.5F}));
    }
    EXPECT_NEAR(step({.vx = 0.5F}).vx, 0.5F, 1e-4F);
    EXPECT_FLOAT_EQ(step({.vx = 0.5F}).vx, 0.5F);
    // Stopped at 2 m/s²: 0.01 m/s a cycle.
    EXPECT_NEAR(step({.vx = 0.0F}).vx, 0.49F, 1e-4F);
    // Reversed: down to zero at the slowing limit, then up at the other.
    float vx = 0.49F;
    int cycles = 0;
    while (vx > 0.0F && cycles < 100) {
        vx = step({.vx = -0.2F}).vx;
        ++cycles;
    }
    EXPECT_GE(cycles, 49);  // 0.49 m/s at 0.01 m/s a cycle, give or take rounding
    EXPECT_LE(cycles, 50);
    EXPECT_FLOAT_EQ(vx, 0.0F);
    EXPECT_NEAR(step({.vx = -0.2F}).vx, -0.005F, 1e-6F);
    // vy alike; the yaw rate is not limited.
    const auto turning = step({.vx = -0.005F, .vy = 0.3F, .yaw_rate = 1.0F});
    EXPECT_NEAR(turning.vy, 0.005F, 1e-6F);
    EXPECT_FLOAT_EQ(turning.yaw_rate, 1.0F);
}

TEST(ControlChain, WithoutLimitsOrClosedLoopsTheSetpointPassesThrough)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    EXPECT_FLOAT_EQ(chain.step(dt, {.vx = 0.5F}, 0.0F, true, open_loop, max_speed, targets).demand.vx, 0.5F);
    EXPECT_FLOAT_EQ(chain.step(dt, {.vx = -0.5F}, 0.0F, true, open_loop, max_speed, targets).demand.vx, -0.5F);
    // A limit only on growing: slowing is immediate, across zero too.
    const VelocityLimits growing{.acc_max = 1.0F};
    EXPECT_FLOAT_EQ(chain.step(dt, {.vx = 0.0F}, 0.0F, true, open_loop, max_speed, targets, growing).demand.vx,
                    0.0F);
    // Open loop: nothing drives, so nothing is limited, and the limited
    // setpoint starts from zero when the loops close again.
    const VelocityLimits limits{.acc_max = 1.0F, .dec_max = 1.0F};
    EXPECT_FLOAT_EQ(chain.step(dt, {.vx = 0.5F}, 0.0F, false, open_loop, max_speed, targets, limits).demand.vx,
                    0.5F);
    EXPECT_NEAR(chain.step(dt, {.vx = 0.5F}, 0.0F, true, open_loop, max_speed, targets, limits).demand.vx, 0.005F,
                1e-6F);
}

TEST(ControlChain, AtRestTheIntegralIsHeldAtZero)
{
    ControlChain chain(&diff_2wd);
    // A skid while driving: the gyro sees the body turn, and the integral grows.
    static_cast<void>(run(chain, {.vx = 0.5F}, -0.5F, true, pi, 100));
    ASSERT_GT(chain.yaw_integral(), 0.1F);
    // Stopped, with the setpoint zero on every axis: no integral, so the
    // demand only resists the measured turn.
    const auto stopped = run(chain, {}, -0.2F, true, pi);
    EXPECT_FLOAT_EQ(chain.yaw_integral(), 0.0F);
    EXPECT_NEAR(stopped.demand.yaw_rate, 0.5F * 0.2F, 1e-6F);
    // Still slowing down under a limit is not at rest yet.
    Targets targets{};
    const VelocityLimits limits{.acc_max = 1.0F, .dec_max = 1.0F};
    for (int i = 0; i < 100; ++i) {
        static_cast<void>(chain.step(dt, {.vx = 0.5F}, -0.5F, true, pi, max_speed, targets, limits));
    }
    const float driving = chain.yaw_integral();
    static_cast<void>(chain.step(dt, {}, -0.5F, true, pi, max_speed, targets, limits));
    EXPECT_GT(chain.yaw_integral(), driving);
}

}  // namespace
