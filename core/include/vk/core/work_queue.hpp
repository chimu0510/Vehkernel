// SPDX-License-Identifier: GPL-3.0-or-later
// Work queues and work items (ADR-0008), with the timing monitor (ADR-0015).
//
// Modules do not own tasks. Each is a work item attached to one of a few
// rate-tiered work queues, each queue one FreeRTOS task, and runs when its
// timer period elapses or when a topic it listens to is published. ISRs
// never publish; they schedule the work item that does.
//
// Every run is timed: run time, start delay (jitter) and overruns are kept
// per work item and published on the work_item_status topic.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/hal/clock.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace vk::core {

class WorkQueue;

// Timing statistics of one work item since boot.
struct WorkItemStats {
    std::uint32_t runs;
    std::uint32_t overruns;          // runs that did not finish before the next was due
    std::uint32_t run_time_last_us;
    std::uint32_t run_time_max_us;
    std::uint32_t run_time_mean_us;
    std::uint32_t jitter_max_us;     // largest delay from due (or scheduled) time to start
};

class WorkItem {
public:
    WorkItem(std::string_view name, WorkQueue& queue);
    virtual ~WorkItem();

    WorkItem(const WorkItem&) = delete;
    WorkItem& operator=(const WorkItem&) = delete;
    WorkItem(WorkItem&&) = delete;
    WorkItem& operator=(WorkItem&&) = delete;

    // Runs every `interval_us`, the first time one interval from now.
    void schedule_periodic(std::uint32_t interval_us);

    // Runs each time `topic` is published.
    void trigger_on(TopicBase& topic);

    // Runs once, soon. From a task.
    void schedule_now();

    // Runs once, soon. From an ISR at a priority allowed to call FreeRTOS.
    void schedule_now_from_isr();

    [[nodiscard]] std::string_view name() const { return name_; }
    [[nodiscard]] const WorkQueue& queue() const { return *queue_; }
    [[nodiscard]] WorkItemStats stats() const;

    // Position in the list of all work items; also its work_item_status instance.
    [[nodiscard]] std::size_t index() const { return index_; }

protected:
    virtual void run() = 0;

private:
    friend class WorkQueue;
    friend class TopicBase;

    void record_run(std::uint64_t start_us, std::uint64_t end_us, std::uint64_t reference_us);

    std::string_view name_;
    WorkQueue* queue_;
    std::size_t index_ = 0;
    WorkItem* next_in_queue_ = nullptr;
    WorkItem* next_listener_ = nullptr;
    TopicBase* topic_ = nullptr;

    std::atomic<bool> pending_{false};
    std::uint64_t scheduled_at_us_ = 0;
    std::uint32_t interval_us_ = 0;  // 0 when not periodic
    std::uint64_t next_due_us_ = 0;

    std::uint32_t runs_ = 0;
    std::uint32_t overruns_ = 0;
    std::uint32_t run_time_last_us_ = 0;
    std::uint32_t run_time_max_us_ = 0;
    std::uint64_t run_time_total_us_ = 0;
    std::uint32_t jitter_max_us_ = 0;
};

class WorkQueue {
public:
    using Clock = std::uint64_t (*)() noexcept;

    // `priority` is a FreeRTOS task priority. `clock` is replaceable for tests.
    constexpr WorkQueue(std::string_view name, unsigned priority, Clock clock = &hal::SystemClock::now_us)
        : name_(name), priority_(priority), clock_(clock)
    {
    }

    WorkQueue(const WorkQueue&) = delete;
    WorkQueue& operator=(const WorkQueue&) = delete;
    WorkQueue(WorkQueue&&) = delete;
    WorkQueue& operator=(WorkQueue&&) = delete;
    ~WorkQueue() = default;

    // Runs, once each and in attach order, every work item that is scheduled
    // or due. The queue's task calls this each time it wakes.
    void process();

    // When the earliest periodic work item is due; 0 if none is periodic.
    [[nodiscard]] std::uint64_t next_due_us() const;

    [[nodiscard]] std::string_view name() const { return name_; }
    [[nodiscard]] unsigned priority() const { return priority_; }
    [[nodiscard]] std::uint64_t now_us() const { return clock_(); }

private:
    friend class WorkItem;
    friend void start_work_queues();

    void attach(WorkItem& item);
    void detach(WorkItem& item);
    void wake();
    void wake_from_isr();
    [[noreturn]] void task_loop();

    std::string_view name_;
    unsigned priority_;
    Clock clock_;
    WorkItem* first_ = nullptr;
    void* task_ = nullptr;  // FreeRTOS task handle once started
};

// The rate tiers, highest priority first.
enum class WorkQueueTier : std::uint8_t {
    Ctrl,  // wq:ctrl, the control path: IMU, estimation, control, output
    Io,    // wq:io, Links and other I/O
    Slow,  // wq:slow, housekeeping such as the timing monitor
};

WorkQueue& work_queue(WorkQueueTier tier);

// Creates the task of every tier. Called by start_scheduler().
void start_work_queues();

// Every work item that exists, in creation order.
std::span<WorkItem* const> work_items();

// The most work items that may exist at once: one work_item_status instance each.
inline constexpr std::size_t max_work_items = 16;

}  // namespace vk::core
