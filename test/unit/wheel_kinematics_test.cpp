// SPDX-License-Identifier: GPL-3.0-or-later
// The wheel kinematics of the WHEELED vehicle model (ADR-0013, ADR-0014):
// J and J⁺ against the closed forms of differential and skid-steer frames.
#include <vk/estimation/models/wheel_kinematics.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>

namespace {

using vk::estimation::Twist;
using vk::estimation::TwistAxis;
using vk::estimation::WheelGeometry;
using vk::estimation::WheelKinematics;

constexpr float radius = 0.05F;
constexpr float track = 0.2F;

WheelKinematics diff_2wd()
{
    const std::array<WheelGeometry, 2> wheels{
        {{.y = -track / 2, .radius = radius}, {.y = track / 2, .radius = radius}}};
    return WheelKinematics(wheels);
}

WheelKinematics skid_4wd()
{
    const std::array<WheelGeometry, 4> wheels{{{.x = 0.1F, .y = -track / 2, .radius = radius},
                                               {.x = 0.1F, .y = track / 2, .radius = radius},
                                               {.x = -0.1F, .y = -track / 2, .radius = radius},
                                               {.x = -0.1F, .y = track / 2, .radius = radius}}};
    return WheelKinematics(wheels);
}

std::uint8_t bits(TwistAxis axis)
{
    return static_cast<std::uint8_t>(axis);
}

void expect_twist(const Twist& actual, const Twist& expected, float tolerance = 1e-5F)
{
    EXPECT_NEAR(actual.vx, expected.vx, tolerance);
    EXPECT_NEAR(actual.vy, expected.vy, tolerance);
    EXPECT_NEAR(actual.yaw_rate, expected.yaw_rate, tolerance);
}

TEST(WheelKinematics, Diff2wdMatrixIsTheClosedForm)
{
    const WheelKinematics k = diff_2wd();
    ASSERT_TRUE(k.valid());
    ASSERT_EQ(k.size(), 2U);
    // Left: (vx + yaw_rate·track/2)/r; right: (vx - yaw_rate·track/2)/r.
    EXPECT_FLOAT_EQ(k.row(0)[0], 1.0F / radius);
    EXPECT_FLOAT_EQ(k.row(0)[1], 0.0F);
    EXPECT_FLOAT_EQ(k.row(0)[2], track / 2 / radius);
    EXPECT_FLOAT_EQ(k.row(1)[0], 1.0F / radius);
    EXPECT_FLOAT_EQ(k.row(1)[2], -track / 2 / radius);
    EXPECT_TRUE(k.moves(TwistAxis::Vx));
    EXPECT_FALSE(k.moves(TwistAxis::Vy));
    EXPECT_TRUE(k.moves(TwistAxis::YawRate));
}

TEST(WheelKinematics, ForwardAndRightTurnSpinTheWheelsAsExpected)
{
    const WheelKinematics k = diff_2wd();
    std::array<float, 2> w{};
    k.wheel_speeds({.vx = 0.5F}, w);
    EXPECT_FLOAT_EQ(w[0], 10.0F);
    EXPECT_FLOAT_EQ(w[1], 10.0F);
    // A positive yaw rate turns right (z down): the left wheel runs faster.
    k.wheel_speeds({.yaw_rate = 1.0F}, w);
    EXPECT_FLOAT_EQ(w[0], 2.0F);
    EXPECT_FLOAT_EQ(w[1], -2.0F);
    // vy is ignored by plain wheels.
    k.wheel_speeds({.vy = 1.0F}, w);
    EXPECT_FLOAT_EQ(w[0], 0.0F);
    EXPECT_FLOAT_EQ(w[1], 0.0F);
}

TEST(WheelKinematics, PseudoInverseIsTheClosedForm)
{
    const WheelKinematics k = diff_2wd();
    // vx = r·(wl + wr)/2, yaw_rate = r·(wl - wr)/track.
    const std::array<float, 2> w{12.0F, 8.0F};
    const auto m = k.measure(w);
    expect_twist(m.twist, {.vx = radius * 10.0F, .vy = 0.0F, .yaw_rate = radius * 4.0F / track});
    // Two wheels fit any pair of speeds.
    EXPECT_NEAR(m.residual, 0.0F, 1e-6F);
}

TEST(WheelKinematics, Skid4wdMatchesTheDifferentialMath)
{
    const WheelKinematics skid = skid_4wd();
    ASSERT_TRUE(skid.valid());
    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(skid.row(i)[0], 1.0F / radius);
        EXPECT_FLOAT_EQ(skid.row(i)[2], (i % 2 == 0 ? 1.0F : -1.0F) * track / 2 / radius);
    }
    const std::array<float, 4> w{12.0F, 8.0F, 12.0F, 8.0F};
    expect_twist(skid.measure(w).twist, diff_2wd().measure(std::span(w).first(2)).twist);
}

TEST(WheelKinematics, RoundTripRecoversTheTwist)
{
    const std::array<Twist, 4> twists{{{.vx = 0.4F}, {.yaw_rate = -1.5F}, {.vx = -0.3F, .yaw_rate = 0.8F},
                                       {.vx = 1.2F, .vy = 0.0F, .yaw_rate = 2.5F}}};
    for (const WheelKinematics& k : {diff_2wd(), skid_4wd()}) {
        for (const Twist& t : twists) {
            std::array<float, 4> w{};
            k.wheel_speeds(t, w);
            const auto m = k.measure(std::span(w).first(k.size()));
            expect_twist(m.twist, t);
            EXPECT_NEAR(m.residual, 0.0F, 1e-6F);
        }
    }
}

TEST(WheelKinematics, MecanumRollersMoveSideways)
{
    // A mecanum frame with rollers at ±45°: J has full rank, vy included.
    constexpr float a = std::numbers::pi_v<float> / 4;
    const std::array<WheelGeometry, 4> wheels{{{.x = 0.1F, .y = -0.1F, .radius = radius, .roller_angle = a},
                                               {.x = 0.1F, .y = 0.1F, .radius = radius, .roller_angle = -a},
                                               {.x = -0.1F, .y = -0.1F, .radius = radius, .roller_angle = -a},
                                               {.x = -0.1F, .y = 0.1F, .radius = radius, .roller_angle = a}}};
    const WheelKinematics k(wheels);
    ASSERT_TRUE(k.valid());
    EXPECT_TRUE(k.moves(TwistAxis::Vy));
    const Twist t{.vx = 0.2F, .vy = -0.3F, .yaw_rate = 0.7F};
    std::array<float, 4> w{};
    k.wheel_speeds(t, w);
    expect_twist(k.measure(w).twist, t, 1e-4F);
}

TEST(WheelKinematics, ResidualGrowsWithDisagreement)
{
    const WheelKinematics k = skid_4wd();
    std::array<float, 4> w{};
    k.wheel_speeds({.vx = 0.5F, .yaw_rate = 0.5F}, w);
    EXPECT_NEAR(k.measure(w).residual, 0.0F, 1e-6F);
    // One wheel slips: the front left spins 1 and then 2 rad/s too fast.
    w[0] += 1.0F;
    const float small = k.measure(w).residual;
    w[0] += 1.0F;
    const float large = k.measure(w).residual;
    EXPECT_GT(small, 1e-3F);
    EXPECT_GT(large, small);
    // Its rim speed is 2·r too fast; the fit explains part of it.
    EXPECT_LT(large, 2.0F * radius);
}

TEST(WheelKinematics, SaturationScalesUniformly)
{
    const WheelKinematics k = diff_2wd();
    std::array<float, 2> w{};
    const Twist demand{.vx = 1.0F, .yaw_rate = 2.0F};  // wheels 24 and 16 rad/s
    const auto fits = k.allocate(demand, 30.0F, w);
    EXPECT_FLOAT_EQ(fits.scale, 1.0F);
    EXPECT_EQ(fits.limited, 0);
    const auto limited = k.allocate(demand, 12.0F, w);
    EXPECT_FLOAT_EQ(limited.scale, 0.5F);
    EXPECT_FLOAT_EQ(w[0], 12.0F);
    EXPECT_FLOAT_EQ(w[1], 8.0F);
    EXPECT_EQ(limited.limited, bits(TwistAxis::Vx) | bits(TwistAxis::YawRate));
    // The ratio of vx to yaw rate, the path's curvature, is kept.
    const auto m = k.measure(w);
    EXPECT_NEAR(m.twist.vx / m.twist.yaw_rate, demand.vx / demand.yaw_rate, 1e-5F);
}

TEST(WheelKinematics, SaturationReportsOnlyAxesWithADemand)
{
    const WheelKinematics k = diff_2wd();
    std::array<float, 2> w{};
    EXPECT_EQ(k.allocate({.yaw_rate = 10.0F}, 5.0F, w).limited, bits(TwistAxis::YawRate));
    // vy is not an axis the frame moves: never limited.
    EXPECT_EQ(k.allocate({.vx = 2.0F, .vy = 1.0F}, 5.0F, w).limited, bits(TwistAxis::Vx));
    // A zero limit stops every wheel.
    const auto stop = k.allocate({.vx = 1.0F}, 0.0F, w);
    EXPECT_FLOAT_EQ(stop.scale, 0.0F);
    EXPECT_FLOAT_EQ(w[0], 0.0F);
    EXPECT_FLOAT_EQ(w[1], 0.0F);
}

TEST(WheelKinematics, InvalidGeometryIsRefused)
{
    const std::array<WheelGeometry, 2> no_track{{{.radius = radius}, {.radius = radius}}};
    EXPECT_FALSE(WheelKinematics(no_track).valid());
    const std::array<WheelGeometry, 2> no_radius{{{.y = -0.1F}, {.y = 0.1F}}};
    EXPECT_FALSE(WheelKinematics(no_radius).valid());
    EXPECT_FALSE(WheelKinematics(std::span<const WheelGeometry>{}).valid());
    EXPECT_FALSE(WheelKinematics().valid());
}

}  // namespace
