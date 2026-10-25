// SPDX-License-Identifier: GPL-3.0-or-later
// The control chain (ADR-0013): a body velocity setpoint through the vehicle
// model to a wheel speed target per output.
#include <vk/control/control_chain.hpp>
#include <vk/msg/enums.hpp>
#include <vk/vehicle_model/wheeled_model.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace {

using vk::control::ControlChain;
using vk::msg::FrameType;
using vk::vehicle_model::max_outputs;
using vk::vehicle_model::WheeledModel;

using Targets = std::array<float, max_outputs>;

const WheeledModel diff_2wd({.type = FrameType::Diff2wd, .radius = 0.05F, .track = 0.2F, .outputs = {1, 2, 3, 4}});

TEST(ControlChain, ABodyVelocityBecomesWheelSpeeds)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    const auto allocation = chain.step({.vx = 0.5F, .yaw_rate = 1.0F}, 30.0F, targets);
    EXPECT_FLOAT_EQ(allocation.scale, 1.0F);
    EXPECT_FLOAT_EQ(targets[0], 12.0F);
    EXPECT_FLOAT_EQ(targets[1], 8.0F);
    EXPECT_TRUE(std::isnan(targets[2]));
    EXPECT_TRUE(std::isnan(targets[3]));
}

TEST(ControlChain, ATwistBeyondReachIsScaled)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    const auto allocation = chain.step({.vx = 1.0F, .yaw_rate = 2.0F}, 12.0F, targets);
    EXPECT_FLOAT_EQ(allocation.scale, 0.5F);
    EXPECT_FLOAT_EQ(targets[0], 12.0F);
    EXPECT_FLOAT_EQ(targets[1], 8.0F);
}

TEST(ControlChain, NoSetpointOrNoModelDrivesNothing)
{
    ControlChain chain(&diff_2wd);
    Targets targets{};
    static_cast<void>(chain.step({.vx = NAN, .vy = NAN, .yaw_rate = NAN}, 30.0F, targets));
    for (const float target : targets) {
        EXPECT_TRUE(std::isnan(target));
    }
    ControlChain none(nullptr);
    EXPECT_FALSE(none.has_model());
    static_cast<void>(none.step({.vx = 0.5F}, 30.0F, targets));
    for (const float target : targets) {
        EXPECT_TRUE(std::isnan(target));
    }
}

}  // namespace
