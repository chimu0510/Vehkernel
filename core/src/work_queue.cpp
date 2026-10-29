// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/core/work_queue.hpp>

#include <vk/core/critical_section.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

namespace vk::core {
namespace {

std::array<WorkItem*, max_work_items> items{};
std::size_t item_count = 0;

std::uint32_t clamp_us(std::uint64_t value)
{
    return static_cast<std::uint32_t>(std::min<std::uint64_t>(value, std::numeric_limits<std::uint32_t>::max()));
}

// Ticks to wait before checking the clock again. Due times follow the
// microsecond clock, but sleeping counts ticks, and ticks may run slow against
// the clock (the POSIX port's run about 5 % slow). So a long wait stops short
// by an eighth and re-checks, and a short one rounds down; the queue then
// never wakes later than about one tick past a due time.
TickType_t ticks_for(std::uint64_t remaining_us)
{
    constexpr std::uint64_t us_per_tick = 1'000'000 / configTICK_RATE_HZ;
    constexpr std::uint64_t short_wait_ticks = 8;
    std::uint64_t ticks = remaining_us / us_per_tick;
    if (ticks > short_wait_ticks) {
        ticks -= ticks / 8;
    }
    return static_cast<TickType_t>(ticks == 0 ? 1 : ticks);
}

}  // namespace

// --- WorkItem ---------------------------------------------------------------

WorkItem::WorkItem(std::string_view name, WorkQueue& queue) : name_(name), queue_(&queue)
{
    {
        const CriticalSection lock;
        if (item_count == items.size()) {
            fail("too many work items");
        }
        index_ = item_count;
        items.at(item_count++) = this;
    }
    queue.attach(*this);
}

WorkItem::~WorkItem()
{
    if (topic_ != nullptr) {
        topic_->remove_listener(*this);
    }
    queue_->detach(*this);
    const CriticalSection lock;
    // Keep creation order; later items move down one place.
    for (std::size_t i = index_; i + 1 < item_count; ++i) {
        items.at(i) = items.at(i + 1);
        items.at(i)->index_ = i;
    }
    items.at(--item_count) = nullptr;
}

void WorkItem::schedule_periodic(std::uint32_t interval_us)
{
    const std::uint64_t now = queue_->now_us();
    {
        const CriticalSection lock;
        interval_us_ = interval_us;
        next_due_us_ = now + interval_us;
    }
    queue_->wake();
}

void WorkItem::trigger_on(TopicBase& topic)
{
    if (topic_ != nullptr) {
        fail("a work item listens to at most one topic");
    }
    topic_ = &topic;
    topic.add_listener(*this);
}

void WorkItem::schedule_now()
{
    scheduled_at_us_ = queue_->now_us();
    pending_.store(true);
    queue_->wake();
}

void WorkItem::schedule_now_from_isr()
{
    scheduled_at_us_ = queue_->now_us();
    pending_.store(true);
    queue_->wake_from_isr();
}

WorkItemStats WorkItem::stats() const
{
    const CriticalSection lock;
    return {.runs = runs_,
            .overruns = overruns_,
            .run_time_last_us = run_time_last_us_,
            .run_time_max_us = run_time_max_us_,
            .run_time_mean_us = runs_ == 0 ? 0 : clamp_us(run_time_total_us_ / runs_),
            .jitter_max_us = jitter_max_us_,
            .periodic = interval_us_ != 0};
}

void WorkItem::record_run(std::uint64_t start_us, std::uint64_t end_us, std::uint64_t reference_us)
{
    const std::uint32_t run_time = clamp_us(end_us - start_us);
    const std::uint32_t jitter = start_us > reference_us ? clamp_us(start_us - reference_us) : 0;
    const CriticalSection lock;
    ++runs_;
    run_time_last_us_ = run_time;
    run_time_max_us_ = std::max(run_time_max_us_, run_time);
    run_time_total_us_ += run_time;
    jitter_max_us_ = std::max(jitter_max_us_, jitter);
}

// --- Topic listeners --------------------------------------------------------

void TopicBase::add_listener(WorkItem& item)
{
    const CriticalSection lock;
    item.next_listener_ = first_listener_;
    first_listener_ = &item;
}

void TopicBase::remove_listener(WorkItem& item)
{
    const CriticalSection lock;
    for (WorkItem** link = &first_listener_; *link != nullptr; link = &(*link)->next_listener_) {
        if (*link == &item) {
            *link = item.next_listener_;
            item.next_listener_ = nullptr;
            return;
        }
    }
}

void TopicBase::notify_listeners()
{
    // Listeners are added at start-up and removed only when destroyed, so the
    // list is walked under the lock; schedule_now() never blocks.
    const CriticalSection lock;
    for (WorkItem* item = first_listener_; item != nullptr; item = item->next_listener_) {
        item->schedule_now();
    }
}

// --- WorkQueue --------------------------------------------------------------

void WorkQueue::attach(WorkItem& item)
{
    const CriticalSection lock;
    WorkItem** link = &first_;
    while (*link != nullptr) {
        link = &(*link)->next_in_queue_;
    }
    *link = &item;
}

void WorkQueue::detach(WorkItem& item)
{
    const CriticalSection lock;
    for (WorkItem** link = &first_; *link != nullptr; link = &(*link)->next_in_queue_) {
        if (*link == &item) {
            *link = item.next_in_queue_;
            return;
        }
    }
}

void WorkQueue::wake()
{
    if (task_ != nullptr) {
        xTaskNotifyGive(static_cast<TaskHandle_t>(task_));
    }
}

void WorkQueue::wake_from_isr()
{
    if (task_ != nullptr) {
        BaseType_t higher_priority_woken = pdFALSE;
        vTaskNotifyGiveFromISR(static_cast<TaskHandle_t>(task_), &higher_priority_woken);
        portYIELD_FROM_ISR(higher_priority_woken);
    }
}

std::uint64_t WorkQueue::next_due_us() const
{
    const CriticalSection lock;
    std::uint64_t earliest = 0;
    for (const WorkItem* item = first_; item != nullptr; item = item->next_in_queue_) {
        if (item->interval_us_ != 0 && (earliest == 0 || item->next_due_us_ < earliest)) {
            earliest = item->next_due_us_;
        }
    }
    return earliest;
}

void WorkQueue::process()
{
    WorkItem* item = nullptr;
    {
        const CriticalSection lock;
        item = first_;
    }
    while (item != nullptr) {
        const std::uint64_t now = clock_();
        bool due = false;
        std::uint64_t due_at = 0;
        {
            const CriticalSection lock;
            due = item->interval_us_ != 0 && now >= item->next_due_us_;
            due_at = item->next_due_us_;
        }
        const bool scheduled = item->pending_.exchange(false);
        if (due || scheduled) {
            const std::uint64_t reference = due ? due_at : item->scheduled_at_us_;
            const std::uint64_t start = clock_();
            item->run();
            const std::uint64_t end = clock_();
            item->record_run(start, end, reference);

            const CriticalSection lock;
            bool overran = false;
            if (due) {
                item->next_due_us_ += item->interval_us_;
                // Skip periods the run used up; each skipped period is one overrun.
                while (item->next_due_us_ <= end) {
                    item->next_due_us_ += item->interval_us_;
                    overran = true;
                }
            }
            // Triggered again before this run finished: it could not keep up.
            if (!due && item->pending_.load()) {
                overran = true;
            }
            if (overran) {
                ++item->overruns_;
            }
        }
        const CriticalSection lock;
        item = item->next_in_queue_;
    }
}

void WorkQueue::task_loop()
{
    {
        // Periods start when the queue starts: time spent before the
        // scheduler ran is neither jitter nor an overrun.
        const std::uint64_t now = clock_();
        const CriticalSection lock;
        for (WorkItem* item = first_; item != nullptr; item = item->next_in_queue_) {
            if (item->interval_us_ != 0 && item->next_due_us_ < now) {
                item->next_due_us_ = now;
            }
        }
    }
    for (;;) {
        TickType_t wait = portMAX_DELAY;
        const std::uint64_t due = next_due_us();
        if (due != 0) {
            const std::uint64_t now = clock_();
            wait = due > now ? ticks_for(due - now) : 0;
        }
        ulTaskNotifyTake(pdTRUE, wait);
        process();
    }
}

// --- Tiers ------------------------------------------------------------------

namespace {

// Null-terminated, since they also name the FreeRTOS tasks.
constexpr std::array<const char*, 3> tier_names{"wq:ctrl", "wq:io", "wq:slow"};

constinit std::array<WorkQueue, 3> tiers{{
    WorkQueue(tier_names[0], configMAX_PRIORITIES - 2),
    WorkQueue(tier_names[1], configMAX_PRIORITIES - 4),
    WorkQueue(tier_names[2], configMAX_PRIORITIES - 6),
}};

struct TaskMemory {
    StaticTask_t tcb;
    std::array<StackType_t, vkconfigWORK_QUEUE_STACK_SIZE> stack;
};

vkconfigTASK_STACK_ATTRIBUTE std::array<TaskMemory, 3> task_memory;

}  // namespace

WorkQueue& work_queue(WorkQueueTier tier)
{
    return tiers.at(static_cast<std::size_t>(tier));
}

void start_work_queues()
{
    for (std::size_t i = 0; i < tiers.size(); ++i) {
        WorkQueue& queue = tiers.at(i);
        if (queue.task_ != nullptr) {
            continue;
        }
        TaskMemory& memory = task_memory.at(i);
        queue.task_ = xTaskCreateStatic(
            [](void* self) { static_cast<WorkQueue*>(self)->task_loop(); },
            tier_names.at(i), memory.stack.size(), &queue, queue.priority_, memory.stack.data(), &memory.tcb);
    }
}

std::span<WorkItem* const> work_items()
{
    return {items.data(), item_count};
}

}  // namespace vk::core
