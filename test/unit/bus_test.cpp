// The bus at its public interface (ADR-0008): publications, subscriptions,
// queued topics, instances and the topic list.
#include <vk/core/bus.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/registry.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

using vk::core::Publication;
using vk::core::Subscription;

// Hand-written messages shaped like generated ones. Each test uses its own
// Tag, so each gets a fresh topic: topic storage is static.
template <int Tag, std::size_t Queue = 1, std::size_t Instances = 1>
struct TestMessage {
    std::uint64_t timestamp{};
    std::int32_t value{};

    static constexpr std::string_view topic_name = "test_message";
    static constexpr std::uint64_t schema_hash = 0;
    static constexpr std::size_t queue_length = Queue;
    static constexpr std::size_t instances = Instances;
    static constexpr float nominal_rate_hz = 0.0F;
};

template <class T>
T message(std::uint64_t timestamp, std::int32_t value)
{
    T m{};
    m.timestamp = timestamp;
    m.value = value;
    return m;
}

TEST(Bus, SubscriptionSeesNothingBeforeTheFirstPublication)
{
    using M = TestMessage<1>;
    Subscription<M> sub;
    M out{};
    EXPECT_FALSE(sub.updated());
    EXPECT_FALSE(sub.copy(out));
}

TEST(Bus, CopyReturnsTheLatestValue)
{
    using M = TestMessage<2>;
    Publication<M> pub;
    Subscription<M> sub;
    pub.publish(message<M>(10, 1));
    pub.publish(message<M>(20, 2));
    ASSERT_TRUE(sub.updated());
    M out{};
    ASSERT_TRUE(sub.copy(out));
    EXPECT_EQ(out.value, 2);
    EXPECT_EQ(out.timestamp, 20U);
}

TEST(Bus, UpdatedTurnsFalseAfterCopyAndTrueOnTheNextPublication)
{
    using M = TestMessage<3>;
    Publication<M> pub;
    Subscription<M> sub;
    M out{};
    pub.publish(message<M>(1, 1));
    ASSERT_TRUE(sub.copy(out));
    EXPECT_FALSE(sub.updated());
    EXPECT_FALSE(sub.copy(out));
    pub.publish(message<M>(2, 2));
    EXPECT_TRUE(sub.updated());
}

TEST(Bus, SubscriptionCreatedLaterStillSeesTheLatestValue)
{
    using M = TestMessage<4>;
    Publication<M> pub;
    pub.publish(message<M>(5, 42));
    Subscription<M> late;
    M out{};
    EXPECT_TRUE(late.updated());
    ASSERT_TRUE(late.copy(out));
    EXPECT_EQ(out.value, 42);
}

TEST(Bus, SubscriptionsReadIndependently)
{
    using M = TestMessage<5>;
    Publication<M> pub;
    Subscription<M> first;
    Subscription<M> second;
    M out{};
    pub.publish(message<M>(1, 7));
    ASSERT_TRUE(first.copy(out));
    EXPECT_FALSE(first.updated());
    EXPECT_TRUE(second.updated());
}

TEST(Bus, LatestValueTopicCountsSkippedPublicationsAsMissed)
{
    using M = TestMessage<6>;
    Publication<M> pub;
    Subscription<M> sub;
    M out{};
    for (int i = 0; i < 3; ++i) {
        pub.publish(message<M>(static_cast<std::uint64_t>(i), i));
    }
    ASSERT_TRUE(sub.copy(out));
    EXPECT_EQ(sub.missed(), 2U);
}

TEST(Bus, QueuedTopicDrainsInOrder)
{
    using M = TestMessage<7, 4>;
    Publication<M> pub;
    Subscription<M> sub;
    for (int i = 0; i < 3; ++i) {
        pub.publish(message<M>(static_cast<std::uint64_t>(i), i));
    }
    M out{};
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(sub.copy(out));
        EXPECT_EQ(out.value, i);
    }
    EXPECT_FALSE(sub.copy(out));
    EXPECT_FALSE(sub.updated());
    EXPECT_EQ(sub.missed(), 0U);
}

TEST(Bus, QueuedTopicReportsSamplesOverwrittenBeforeTheyWereRead)
{
    using M = TestMessage<8, 4>;
    Publication<M> pub;
    Subscription<M> sub;
    for (int i = 0; i < 10; ++i) {
        pub.publish(message<M>(static_cast<std::uint64_t>(i), i));
    }
    M out{};
    ASSERT_TRUE(sub.copy(out));
    EXPECT_EQ(out.value, 6);  // The oldest still buffered.
    EXPECT_EQ(sub.missed(), 6U);
    int expected = 7;
    while (sub.copy(out)) {
        EXPECT_EQ(out.value, expected++);
    }
    EXPECT_EQ(expected, 10);
    EXPECT_EQ(sub.missed(), 6U);
}

TEST(Bus, QueuedTopicKeepsDrainingAcrossInterleavedPublications)
{
    using M = TestMessage<9, 2>;
    Publication<M> pub;
    Subscription<M> sub;
    M out{};
    int expected = 0;
    for (int i = 0; i < 20; ++i) {
        pub.publish(message<M>(static_cast<std::uint64_t>(i), i));
        ASSERT_TRUE(sub.copy(out));
        EXPECT_EQ(out.value, expected++);
    }
    EXPECT_EQ(sub.missed(), 0U);
}

TEST(Bus, InstancesAreSeparateTopics)
{
    using M = TestMessage<10, 1, 2>;
    Publication<M> pub0(0);
    Publication<M> pub1(1);
    Subscription<M> sub0(0);
    Subscription<M> sub1(1);
    pub1.publish(message<M>(1, 11));
    EXPECT_FALSE(sub0.updated());
    M out{};
    ASSERT_TRUE(sub1.copy(out));
    EXPECT_EQ(out.value, 11);
    pub0.publish(message<M>(2, 10));
    ASSERT_TRUE(sub0.copy(out));
    EXPECT_EQ(out.value, 10);
}

TEST(Bus, TopicStatusReportsPublicationsLastUpdateAndRate)
{
    using M = TestMessage<11>;
    Publication<M> pub;
    // 11 messages 10 ms apart: 100 Hz.
    for (int i = 0; i <= 10; ++i) {
        pub.publish(message<M>(1'000'000 + static_cast<std::uint64_t>(i) * 10'000, i));
    }
    const auto status = vk::core::topic<M>(0).status();
    EXPECT_EQ(status.name, "test_message");
    EXPECT_EQ(status.instance, 0U);
    EXPECT_EQ(status.publications, 11U);
    EXPECT_EQ(status.last_update_us, 1'100'000U);
    EXPECT_FLOAT_EQ(status.rate_hz, 100.0F);
}

TEST(Bus, TopicStatusOfAnUnusedTopicIsEmpty)
{
    using M = TestMessage<12>;
    const auto status = vk::core::topic<M>(0).status();
    EXPECT_EQ(status.publications, 0U);
    EXPECT_EQ(status.last_update_us, 0U);
    EXPECT_EQ(status.rate_hz, 0.0F);
}

TEST(Bus, TopicListHoldsEveryInstanceOfEverySchemaTopic)
{
    const auto topics = vk::msg::all_topics();
    std::size_t imu_instances = 0;
    for (const auto* topic : topics) {
        if (topic->status().name == "imu_sample") {
            ++imu_instances;
        }
    }
    EXPECT_EQ(imu_instances, vk::msg::ImuSample::instances);
}

TEST(Bus, TopicListReflectsPublications)
{
    Publication<vk::msg::ImuSample> pub(1);
    vk::msg::ImuSample sample{};
    sample.timestamp = 1234;
    pub.publish(sample);
    bool found = false;
    for (const auto* topic : vk::msg::all_topics()) {
        const auto status = topic->status();
        if (status.name == "imu_sample" && status.instance == 1) {
            EXPECT_EQ(status.publications, 1U);
            EXPECT_EQ(status.last_update_us, 1234U);
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

}  // namespace
