// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle description and the WHEELED vehicle model (ADR-0013): the
// model the parameters select, and each wheel on its output.
#include <vk/core/parameters.hpp>
#include <vk/msg/enums.hpp>
#include <vk/params/params.hpp>
#include <vk/vehicle_model/vehicle_description.hpp>
#include <vk/vehicle_model/wheeled_model.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

using vk::estimation::TwistAxis;
using vk::msg::FrameClass;
using vk::msg::FrameType;
using vk::msg::VehicleType;
using vk::vehicle_model::max_outputs;
using vk::vehicle_model::VehicleDescription;
using vk::vehicle_model::VehicleModels;
using vk::vehicle_model::WheeledFrame;
using vk::vehicle_model::WheeledModel;

using Targets = std::array<float, max_outputs>;

// Sets parameters for one test and puts the defaults back afterwards.
class VehicleParameters : public testing::Test {
protected:
    void TearDown() override
    {
        auto& parameters = vk::params::parameters();
        for (std::size_t i = 0; i < parameters.count(); ++i) {
            parameters.set(i, parameters.info(i).default_value);
        }
    }

    static void set(std::string_view name, std::int32_t value)
    {
        ASSERT_EQ(vk::params::parameters().set(name, vk::core::ParamValue::from_int(value)),
                  vk::core::ParamResult::Ok);
    }

    static void set(std::string_view name, float value)
    {
        ASSERT_EQ(vk::params::parameters().set(name, vk::core::ParamValue::from_float(value)),
                  vk::core::ParamResult::Ok);
    }
};

TEST(WheeledModel, Diff2wdDrivesItsTwoOutputs)
{
    const WheeledModel model({.type = FrameType::Diff2wd, .radius = 0.05F, .track = 0.2F, .outputs = {1, 2, 3, 4}});
    ASSERT_TRUE(model.valid());
    Targets targets{};
    const auto allocation = model.allocate({.vx = 0.5F, .yaw_rate = 1.0F}, 30.0F, targets);
    EXPECT_FLOAT_EQ(allocation.scale, 1.0F);
    EXPECT_FLOAT_EQ(targets[0], 12.0F);  // left
    EXPECT_FLOAT_EQ(targets[1], 8.0F);   // right
    // Outputs the frame does not use stay uncontrolled.
    EXPECT_TRUE(std::isnan(targets[2]));
    EXPECT_TRUE(std::isnan(targets[3]));
}

TEST(WheeledModel, OutputMappingPlacesEachWheel)
{
    const WheeledModel model({.type = FrameType::Diff2wd, .radius = 0.05F, .track = 0.2F, .outputs = {4, 2, 0, 0}});
    ASSERT_TRUE(model.valid());
    Targets targets{};
    static_cast<void>(model.allocate({.vx = 0.5F, .yaw_rate = 1.0F}, 30.0F, targets));
    EXPECT_TRUE(std::isnan(targets[0]));
    EXPECT_FLOAT_EQ(targets[1], 8.0F);  // right on output 2
    EXPECT_TRUE(std::isnan(targets[2]));
    EXPECT_FLOAT_EQ(targets[3], 12.0F);  // left on output 4
    // The measurement reads each wheel from its output.
    const Targets speeds{0.0F, 8.0F, 0.0F, 12.0F};
    const auto m = model.measure(speeds, Targets{});
    ASSERT_TRUE(m.has_value());
    const auto twist = m.value_or(vk::estimation::WheelMeasurement{}).twist;
    EXPECT_NEAR(twist.vx, 0.5F, 1e-5F);
    EXPECT_NEAR(twist.yaw_rate, 1.0F, 1e-5F);
}

TEST(WheeledModel, Skid4wdDrivesEachSideTogether)
{
    const WheeledModel model({.type = FrameType::Skid4wd, .radius = 0.05F, .track = 0.2F, .outputs = {1, 2, 3, 4}});
    ASSERT_TRUE(model.valid());
    Targets targets{};
    static_cast<void>(model.allocate({.vx = 0.5F, .yaw_rate = 1.0F}, 30.0F, targets));
    EXPECT_FLOAT_EQ(targets[0], 12.0F);
    EXPECT_FLOAT_EQ(targets[1], 8.0F);
    EXPECT_FLOAT_EQ(targets[2], 12.0F);
    EXPECT_FLOAT_EQ(targets[3], 8.0F);
}

TEST(WheeledModel, SaturationIsUniformAndReported)
{
    const WheeledModel model({.type = FrameType::Diff2wd, .radius = 0.05F, .track = 0.2F, .outputs = {1, 2, 3, 4}});
    Targets targets{};
    const auto allocation = model.allocate({.vx = 1.0F, .yaw_rate = 2.0F}, 12.0F, targets);
    EXPECT_FLOAT_EQ(allocation.scale, 0.5F);
    EXPECT_EQ(allocation.limited,
              static_cast<std::uint8_t>(TwistAxis::Vx) | static_cast<std::uint8_t>(TwistAxis::YawRate));
    EXPECT_FLOAT_EQ(targets[0], 12.0F);
    EXPECT_FLOAT_EQ(targets[1], 8.0F);
}

TEST(WheeledModel, NonFiniteDemandLeavesEveryOutputUncontrolled)
{
    const WheeledModel model({.type = FrameType::Diff2wd, .radius = 0.05F, .track = 0.2F, .outputs = {1, 2, 3, 4}});
    Targets targets{};
    static_cast<void>(model.allocate({.vx = std::nanf("")}, 30.0F, targets));
    for (const float target : targets) {
        EXPECT_TRUE(std::isnan(target));
    }
    EXPECT_FALSE(model.measure(Targets{std::nanf(""), 0.0F, 0.0F, 0.0F}, Targets{}).has_value());
}

TEST(WheeledModel, InvalidFramesAreRefused)
{
    // Two wheels on one output.
    EXPECT_FALSE(WheeledModel({.type = FrameType::Diff2wd, .outputs = {1, 1, 3, 4}}).valid());
    EXPECT_FALSE(WheeledModel({.type = FrameType::Skid4wd, .outputs = {1, 2, 3, 2}}).valid());
    // An output the board does not have.
    EXPECT_FALSE(WheeledModel({.type = FrameType::Diff2wd, .outputs = {1, 5, 3, 4}}).valid());
    // No geometry.
    EXPECT_FALSE(WheeledModel({.type = FrameType::Diff2wd, .radius = 0.0F}).valid());
    EXPECT_FALSE(WheeledModel({.type = FrameType::Diff2wd, .track = 0.0F}).valid());
    // A model that is not valid drives nothing.
    const WheeledModel invalid({.type = FrameType::Diff2wd, .outputs = {1, 1, 3, 4}});
    Targets targets{};
    static_cast<void>(invalid.allocate({.vx = 1.0F}, 30.0F, targets));
    EXPECT_TRUE(std::isnan(targets[0]));
    EXPECT_FALSE(invalid.measure(Targets{}, Targets{}).has_value());
}

TEST_F(VehicleParameters, DefaultsDescribeADifferentialFrame)
{
    const VehicleDescription description = vk::vehicle_model::description_from_parameters();
    EXPECT_EQ(description.vehicle_type, VehicleType::Ground);
    EXPECT_EQ(description.frame_class, FrameClass::Wheeled);
    EXPECT_EQ(description.wheeled.type, FrameType::Diff2wd);
    EXPECT_FLOAT_EQ(description.wheeled.radius, vk::params::whl.radius);
    EXPECT_FLOAT_EQ(description.wheeled.track, vk::params::whl.track);
    EXPECT_EQ(description.wheeled.outputs, (std::array<std::uint8_t, 4>{1, 2, 3, 4}));
}

TEST_F(VehicleParameters, TheParametersSelectTheModel)
{
    set("FRAME_TYPE", static_cast<std::int32_t>(FrameType::Skid4wd));
    set("WHL_RADIUS", 0.1F);
    set("WHL_TRACK", 0.4F);
    set("WHL_LEFT_OUT", 2);
    set("WHL_RIGHT_OUT", 1);
    set("WHL_REAR_L_OUT", 4);
    set("WHL_REAR_R_OUT", 3);
    VehicleModels models;
    const auto* model = models.select(vk::vehicle_model::description_from_parameters());
    ASSERT_NE(model, nullptr);
    Targets targets{};
    // Turning right in place at 1 rad/s: the left side at +2, the right at -2.
    static_cast<void>(model->allocate({.yaw_rate = 1.0F}, 30.0F, targets));
    EXPECT_FLOAT_EQ(targets[0], -2.0F);
    EXPECT_FLOAT_EQ(targets[1], 2.0F);
    EXPECT_FLOAT_EQ(targets[2], -2.0F);
    EXPECT_FLOAT_EQ(targets[3], 2.0F);
}

TEST_F(VehicleParameters, ADescriptionNoModelDrivesSelectsNone)
{
    set("WHL_RIGHT_OUT", 1);  // both wheels on output 1
    VehicleModels models;
    EXPECT_EQ(models.select(vk::vehicle_model::description_from_parameters()), nullptr);
    VehicleDescription description;
    description.frame_class = FrameClass::Ackermann;  // not compiled in
    EXPECT_EQ(models.select(description), nullptr);
    description = {};
    description.vehicle_type = VehicleType::Surface;
    EXPECT_EQ(models.select(description), nullptr);
}

TEST(VehicleParameterMetadata, FrameValuesFollowTheSchemaEnums)
{
    // FRAME_CLASS and FRAME_TYPE offer the schema's values, by number.
    for (const auto& entry : vk::params::table()) {
        if (entry.name == "FRAME_TYPE") {
            ASSERT_EQ(entry.enum_values.size(), 2U);
            EXPECT_EQ(entry.enum_values[0], static_cast<std::int32_t>(FrameType::Diff2wd));
            EXPECT_EQ(entry.enum_values[1], static_cast<std::int32_t>(FrameType::Skid4wd));
        } else if (entry.name == "FRAME_CLASS") {
            ASSERT_EQ(entry.enum_values.size(), 1U);
            EXPECT_EQ(entry.enum_values[0], static_cast<std::int32_t>(FrameClass::Wheeled));
        } else if (entry.name == "VEHICLE_TYPE") {
            ASSERT_EQ(entry.enum_values.size(), 1U);
            EXPECT_EQ(entry.enum_values[0], static_cast<std::int32_t>(VehicleType::Ground));
        }
    }
}

}  // namespace
