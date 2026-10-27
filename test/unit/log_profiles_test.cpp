// SPDX-License-Identifier: GPL-3.0-or-later
// Log profile tables generated from schema/log_profiles.toml (spec 0011, ADR-0019).
#include <vk/msg/log_profiles.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>

namespace {

using vk::msg::LogProfileId;
using vk::msg::LogRateKind;
using vk::msg::find_log_profile;

TEST(LogProfiles, FindById)
{
    const auto* profile = find_log_profile(LogProfileId::EstimatorReplay);
    ASSERT_NE(profile, nullptr);
    EXPECT_EQ(profile->id, LogProfileId::EstimatorReplay);
    EXPECT_EQ(profile->name, "estimator_replay");

    EXPECT_EQ(find_log_profile(LogProfileId::None), nullptr);
}

TEST(LogProfiles, FindByName)
{
    const auto* by_id = find_log_profile(LogProfileId::EstimatorReplay);
    const auto* by_name = find_log_profile("estimator_replay");
    ASSERT_NE(by_name, nullptr);
    EXPECT_EQ(by_name, by_id);

    EXPECT_EQ(find_log_profile("unknown"), nullptr);
    EXPECT_EQ(find_log_profile(""), nullptr);
}

TEST(LogProfiles, ProfileProperties)
{
    const auto* profile = find_log_profile(LogProfileId::EstimatorReplay);
    ASSERT_NE(profile, nullptr);
    EXPECT_FALSE(profile->description.empty());
    EXPECT_GT(profile->bandwidth, 0.0F);
}

TEST(LogProfiles, EntryCountForBuildPlatform)
{
    const auto* profile = find_log_profile(LogProfileId::EstimatorReplay);
    ASSERT_NE(profile, nullptr);
#ifdef CONFIG_PLATFORM_SITL
    EXPECT_EQ(profile->entries.size(), 28U);
#else
    EXPECT_EQ(profile->entries.size(), 26U);
#endif
}

TEST(LogProfiles, EntriesNameExpectedTopics)
{
    const auto* profile = find_log_profile(LogProfileId::EstimatorReplay);
    ASSERT_NE(profile, nullptr);
    const auto& entries = profile->entries;

    // Entry 0: imu_sample, instance 0, every_sample
    EXPECT_EQ(entries[0].topic_id, vk::msg::ImuSample::topic_id);
    EXPECT_EQ(entries[0].instance, 0);
    EXPECT_EQ(entries[0].rate_kind, LogRateKind::EverySample);

    // Entry 1: body_velocity_measurement, instance 0, every_sample
    EXPECT_EQ(entries[1].topic_id, vk::msg::BodyVelocityMeasurement::topic_id);
    EXPECT_EQ(entries[1].instance, 0);
    EXPECT_EQ(entries[1].rate_kind, LogRateKind::EverySample);

    // Entry 2: vehicle_attitude, instance 0, 50 Hz
    EXPECT_EQ(entries[2].topic_id, vk::msg::VehicleAttitude::topic_id);
    EXPECT_EQ(entries[2].instance, 0);
    EXPECT_EQ(entries[2].rate_kind, LogRateKind::RateHz);
    EXPECT_FLOAT_EQ(entries[2].rate_hz, 50.0F);

    // Entry 3: vehicle_local_odometry, instance 0, 50 Hz
    EXPECT_EQ(entries[3].topic_id, vk::msg::VehicleLocalOdometry::topic_id);
    EXPECT_EQ(entries[3].instance, 0);
    EXPECT_EQ(entries[3].rate_kind, LogRateKind::RateHz);
    EXPECT_FLOAT_EQ(entries[3].rate_hz, 50.0F);

    // Entry 4: actuator_outputs, instance 0, 50 Hz
    EXPECT_EQ(entries[4].topic_id, vk::msg::ActuatorOutputs::topic_id);
    EXPECT_EQ(entries[4].instance, 0);
    EXPECT_EQ(entries[4].rate_kind, LogRateKind::RateHz);
    EXPECT_FLOAT_EQ(entries[4].rate_hz, 50.0F);

    // Entry 5: vehicle_status, instance 0, as_published
    EXPECT_EQ(entries[5].topic_id, vk::msg::VehicleStatus::topic_id);
    EXPECT_EQ(entries[5].instance, 0);
    EXPECT_EQ(entries[5].rate_kind, LogRateKind::AsPublished);

    // Entries 6..21: work_item_status, instances 0..15, as_published
    for (std::uint8_t inst = 0; inst < 16; ++inst) {
        EXPECT_EQ(entries[6 + inst].topic_id, vk::msg::WorkItemStatus::topic_id);
        EXPECT_EQ(entries[6 + inst].instance, inst);
        EXPECT_EQ(entries[6 + inst].rate_kind, LogRateKind::AsPublished);
    }

    // Entries 22..23: link_status, instances 0..1, as_published
    for (std::uint8_t inst = 0; inst < 2; ++inst) {
        EXPECT_EQ(entries[22 + inst].topic_id, vk::msg::LinkStatus::topic_id);
        EXPECT_EQ(entries[22 + inst].instance, inst);
        EXPECT_EQ(entries[22 + inst].rate_kind, LogRateKind::AsPublished);
    }

    // Entry 24: parameter_update, instance 0, as_published
    EXPECT_EQ(entries[24].topic_id, vk::msg::ParameterUpdate::topic_id);
    EXPECT_EQ(entries[24].instance, 0);
    EXPECT_EQ(entries[24].rate_kind, LogRateKind::AsPublished);

    // Entry 25: estimator_restart, instance 0, as_published
    EXPECT_EQ(entries[25].topic_id, vk::msg::EstimatorRestart::topic_id);
    EXPECT_EQ(entries[25].instance, 0);
    EXPECT_EQ(entries[25].rate_kind, LogRateKind::AsPublished);

#ifdef CONFIG_PLATFORM_SITL
    // Entry 26: vehicle_attitude_groundtruth, instance 0, 50 Hz
    EXPECT_EQ(entries[26].topic_id, vk::msg::VehicleAttitudeGroundtruth::topic_id);
    EXPECT_EQ(entries[26].instance, 0);
    EXPECT_EQ(entries[26].rate_kind, LogRateKind::RateHz);
    EXPECT_FLOAT_EQ(entries[26].rate_hz, 50.0F);

    // Entry 27: vehicle_local_odometry_groundtruth, instance 0, 50 Hz
    EXPECT_EQ(entries[27].topic_id, vk::msg::VehicleLocalOdometryGroundtruth::topic_id);
    EXPECT_EQ(entries[27].instance, 0);
    EXPECT_EQ(entries[27].rate_kind, LogRateKind::RateHz);
    EXPECT_FLOAT_EQ(entries[27].rate_hz, 50.0F);
#endif
}

}  // namespace
