// SPDX-License-Identifier: GPL-3.0-or-later
// Work queues, work items and the timing monitor at their public interfaces
// (ADR-0008, ADR-0015). The queues run on a fake clock and are driven by
// calling process(), as their task does when it wakes.
#include <vk/core/bus.hpp>
#include <vk/core/timing_monitor.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/msg/work_item_status.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

using vk::core::Publication;
using vk::core::Subscription;
using vk::core::WorkItem;
using vk::core::WorkQueue;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

class CountingItem final : public WorkItem {
public:
    explicit CountingItem(WorkQueue& queue, std::uint64_t run_time_us = 0, std::string_view name = "counting")
        : WorkItem(name, queue), run_time_us_(run_time_us)
    {
    }

    int runs = 0;

protected:
    void run() override
    {
        ++runs;
        fake_now += run_time_us_;
    }

private:
    std::uint64_t run_time_us_;
};

struct Trigger {
    std::uint64_t timestamp{};

    static constexpr std::string_view topic_name = "trigger";
    static constexpr std::uint64_t schema_hash = 0;
    static constexpr std::size_t queue_length = 1;
    static constexpr std::size_t instances = 1;
    static constexpr float nominal_rate_hz = 0.0F;
};

class WorkQueueTest : public ::testing::Test {
protected:
    void SetUp() override { fake_now = 1'000'000; }

    WorkQueue queue{"wq:test", 1, &fake_clock};
};

TEST_F(WorkQueueTest, PeriodicItemRunsOnlyWhenDue)
{
    CountingItem item(queue);
    item.schedule_periodic(5000);
    queue.process();
    EXPECT_EQ(item.runs, 0);
    fake_now += 4999;
    queue.process();
    EXPECT_EQ(item.runs, 0);
    fake_now += 1;
    queue.process();
    EXPECT_EQ(item.runs, 1);
    queue.process();
    EXPECT_EQ(item.runs, 1);
    fake_now += 5000;
    queue.process();
    EXPECT_EQ(item.runs, 2);
    EXPECT_EQ(item.stats().runs, 2U);
}

TEST_F(WorkQueueTest, NextDueIsTheEarliestPeriodicItem)
{
    EXPECT_EQ(queue.next_due_us(), 0U);
    CountingItem slow(queue);
    CountingItem fast(queue);
    slow.schedule_periodic(10000);
    fast.schedule_periodic(2000);
    EXPECT_EQ(queue.next_due_us(), fake_now + 2000);
}

TEST_F(WorkQueueTest, RecordsRunTime)
{
    CountingItem item(queue, 300);
    item.schedule_periodic(5000);
    fake_now += 5000;
    queue.process();
    const auto stats = item.stats();
    EXPECT_EQ(stats.run_time_last_us, 300U);
    EXPECT_EQ(stats.run_time_max_us, 300U);
    EXPECT_EQ(stats.run_time_mean_us, 300U);
}

TEST_F(WorkQueueTest, RecordsTheLargestStartDelayAsJitter)
{
    CountingItem item(queue);
    item.schedule_periodic(5000);
    fake_now += 5000 + 250;
    queue.process();
    fake_now += 5000 - 250 + 100;
    queue.process();
    EXPECT_EQ(item.stats().jitter_max_us, 250U);
    EXPECT_EQ(item.stats().overruns, 0U);
}

TEST_F(WorkQueueTest, CountsARunLongerThanItsPeriodAsAnOverrun)
{
    CountingItem item(queue, 7000);
    item.schedule_periodic(5000);
    fake_now += 5000;
    queue.process();
    EXPECT_EQ(item.stats().overruns, 1U);
    // The missed period is skipped, not run late.
    EXPECT_GT(queue.next_due_us(), fake_now);
}

TEST_F(WorkQueueTest, TopicTriggeredItemRunsOncePerPublication)
{
    CountingItem item(queue);
    item.trigger_on(vk::core::topic<Trigger>(0));
    Publication<Trigger> pub;
    queue.process();
    EXPECT_EQ(item.runs, 0);
    pub.publish(Trigger{.timestamp = 1});
    queue.process();
    EXPECT_EQ(item.runs, 1);
    queue.process();
    EXPECT_EQ(item.runs, 1);
}

TEST_F(WorkQueueTest, ScheduleNowRunsOnce)
{
    CountingItem item(queue);
    item.schedule_now();
    queue.process();
    queue.process();
    EXPECT_EQ(item.runs, 1);
}

TEST_F(WorkQueueTest, WorkItemsAreListedInCreationOrderWhileTheyExist)
{
    CountingItem first(queue, 0, "first");
    {
        CountingItem second(queue, 0, "second");
        const auto items = vk::core::work_items();
        ASSERT_EQ(items.size(), 2U);
        EXPECT_EQ(items[0]->name(), "first");
        EXPECT_EQ(items[1]->name(), "second");
        EXPECT_EQ(second.index(), 1U);
    }
    EXPECT_EQ(vk::core::work_items().size(), 1U);
}

TEST_F(WorkQueueTest, TimingMonitorPublishesEachWorkItemOnItsOwnInstance)
{
    CountingItem item(queue, 120, "worker");
    item.schedule_periodic(1000);
    fake_now += 1000;
    queue.process();
    vk::core::TimingMonitor monitor(queue);
    monitor.publish_all();

    Subscription<vk::msg::WorkItemStatus> sub(item.index());
    vk::msg::WorkItemStatus status{};
    ASSERT_TRUE(sub.copy(status));
    EXPECT_EQ(status.work_item, item.index());
    const std::string_view name(reinterpret_cast<const char*>(status.name.data()), 6);
    EXPECT_EQ(name, "worker");
    EXPECT_EQ(status.name[6], 0);  // zero-padded
    EXPECT_EQ(status.runs, 1U);
    EXPECT_EQ(status.run_time_last_us, 120U);
    EXPECT_EQ(status.overruns, 0U);
    EXPECT_EQ(status.timestamp, fake_now);

    Subscription<vk::msg::WorkItemStatus> own(monitor.index());
    EXPECT_TRUE(own.updated());
}

TEST_F(WorkQueueTest, TimingMonitorRunsOnItsPeriod)
{
    vk::core::TimingMonitor monitor(queue, 1'000'000);
    Subscription<vk::msg::WorkItemStatus> sub(monitor.index());
    vk::msg::WorkItemStatus status{};
    while (sub.copy(status)) {
    }
    fake_now += 1'000'000;
    queue.process();
    ASSERT_TRUE(sub.copy(status));
    EXPECT_EQ(status.timestamp, fake_now);
}

}  // namespace
