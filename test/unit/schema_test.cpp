// Topics generated from schema/msg at build time (ADR-0006).
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/work_item_status.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace {

using vk::msg::ImuSample;
using vk::msg::WorkItemStatus;

static_assert(std::is_same_v<decltype(ImuSample::timestamp), std::uint64_t>);
static_assert(offsetof(ImuSample, timestamp) == 0, "the timestamp is always the first field");
static_assert(ImuSample::queue_length == 4);
static_assert(WorkItemStatus::queue_length == 1);

TEST(Schema, CarriesTheTopicName)
{
    EXPECT_EQ(ImuSample::topic_name, "imu_sample");
    EXPECT_EQ(WorkItemStatus::topic_name, "work_item_status");
}

TEST(Schema, GivesEachMessageItsOwnHash)
{
    EXPECT_NE(ImuSample::schema_hash, WorkItemStatus::schema_hash);
}

TEST(Schema, ValueInitialisesEveryField)
{
    const ImuSample sample{};
    EXPECT_EQ(sample.timestamp, 0U);
    EXPECT_EQ(sample.accel[2], 0.0F);
}

}  // namespace
