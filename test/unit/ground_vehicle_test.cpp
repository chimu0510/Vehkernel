// SPDX-License-Identifier: GPL-3.0-or-later
// SITL's ground vehicle (ADR-0018): the pose its wheels' speeds give, on a
// fake clock.
#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/sim/ground_vehicle.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace {

using vk::estimation::WheelGeometry;
using vk::estimation::WheelKinematics;
using vk::sim::GroundVehicle;

constexpr float radius = 0.05F;
constexpr float track = 0.2F;
constexpr std::uint64_t step_us = 5'000;
// The first step after a speed change averages the old and the new twist:
// half a step of the new motion is lost to it.
constexpr float ramp_s = 0.0025F;

std::uint64_t fake_now = 0;
std::array<float, 4> wheel_speeds{};

std::uint64_t now() noexcept
{
    return fake_now;
}

float speed(std::size_t index)
{
    return wheel_speeds.at(index);
}

GroundVehicle::Config diff_2wd(float slip = 0.0F)
{
    const std::array<WheelGeometry, 2> wheels{
        {{.y = -track / 2, .radius = radius}, {.y = track / 2, .radius = radius}}};
    // Left on output 2, right on output 1.
    return {.kinematics = WheelKinematics(wheels), .outputs = {1, 0, 0, 0}, .slip = slip};
}

// Runs the vehicle for `seconds` at the given output speeds.
const GroundVehicle::State& run(GroundVehicle& vehicle, float output1, float output2, float seconds)
{
    wheel_speeds = {output1, output2, 0.0F, 0.0F};
    const auto steps = static_cast<int>(seconds * 1e6F / static_cast<float>(step_us));
    for (int i = 0; i < steps; ++i) {
        fake_now += step_us;
        static_cast<void>(vehicle.state());
    }
    return vehicle.state();
}

class GroundVehicleTest : public testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        wheel_speeds = {};
    }
};

TEST_F(GroundVehicleTest, DrivesStraightNorth)
{
    GroundVehicle vehicle(diff_2wd(), &now, &speed);
    const auto& state = run(vehicle, 10.0F, 10.0F, 2.0F);
    EXPECT_NEAR(state.north, 0.5 * (2.0 - ramp_s), 1e-5);  // 10 rad/s x 0.05 m
    EXPECT_NEAR(state.east, 0.0, 1e-6);
    EXPECT_NEAR(state.yaw, 0.0F, 1e-6F);
    EXPECT_FLOAT_EQ(state.twist.vx, 0.5F);
}

TEST_F(GroundVehicleTest, TurnsRightInPlace)
{
    GroundVehicle vehicle(diff_2wd(), &now, &speed);
    // Left (output 2) forward, right (output 1) backward: 1 rad/s to the right.
    const auto& state = run(vehicle, -2.0F, 2.0F, 1.0F);
    EXPECT_NEAR(state.yaw, 1.0F - ramp_s, 1e-5F);
    EXPECT_NEAR(state.north, 0.0, 1e-6);
    EXPECT_NEAR(state.east, 0.0, 1e-6);
    EXPECT_FLOAT_EQ(state.twist.yaw_rate, 1.0F);
}

TEST_F(GroundVehicleTest, FollowsAnArc)
{
    GroundVehicle vehicle(diff_2wd(), &now, &speed);
    // 0.5 m/s at 0.5 rad/s: a right turn of radius 1 m, a quarter circle in
    // pi s; 3.14 s is whole steps.
    const auto& state = run(vehicle, 9.0F, 11.0F, 3.14F);
    EXPECT_NEAR(state.yaw, 0.5F * (3.14F - ramp_s), 1e-4F);
    EXPECT_NEAR(state.north, 1.0, 3e-3);
    EXPECT_NEAR(state.east, 1.0, 3e-3);
    // Moving on a circle: the centripetal acceleration points right.
    EXPECT_NEAR(state.acceleration.y, 0.25F, 1e-3F);
}

TEST_F(GroundVehicleTest, SlipLosesItsShare)
{
    GroundVehicle vehicle(diff_2wd(0.25F), &now, &speed);
    const auto& forward = run(vehicle, 10.0F, 10.0F, 2.0F);
    EXPECT_NEAR(forward.north, 0.75 * 0.5 * (2.0 - ramp_s), 1e-5);
    const float yaw = forward.yaw;
    const auto& turned = run(vehicle, -2.0F, 2.0F, 1.0F);
    EXPECT_NEAR(turned.yaw - yaw, 0.75F * (1.0F - ramp_s), 1e-5F);
}

TEST_F(GroundVehicleTest, AccelerationFollowsTheWheels)
{
    GroundVehicle vehicle(diff_2wd(), &now, &speed);
    wheel_speeds = {0.0F, 0.0F, 0.0F, 0.0F};
    static_cast<void>(vehicle.state());
    // From rest to 0.5 m/s in one 5 ms step.
    wheel_speeds = {10.0F, 10.0F, 0.0F, 0.0F};
    fake_now += step_us;
    EXPECT_NEAR(vehicle.state().acceleration.x, 100.0F, 1e-2F);
}

}  // namespace
