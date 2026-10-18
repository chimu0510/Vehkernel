// The timing monitor (ADR-0015): publishes every work item's timing
// statistics on the work_item_status topic, one instance per work item.
#pragma once

#include <vk/core/work_queue.hpp>

#include <cstdint>

namespace vk::core {

class TimingMonitor final : public WorkItem {
public:
    static constexpr std::uint32_t default_interval_us = 1'000'000;

    explicit TimingMonitor(WorkQueue& queue, std::uint32_t interval_us = default_interval_us);

    TimingMonitor(const TimingMonitor&) = delete;
    TimingMonitor& operator=(const TimingMonitor&) = delete;
    TimingMonitor(TimingMonitor&&) = delete;
    TimingMonitor& operator=(TimingMonitor&&) = delete;
    ~TimingMonitor() override = default;

    // Publishes the statistics of every work item now.
    void publish_all();

protected:
    void run() override;
};

}  // namespace vk::core
