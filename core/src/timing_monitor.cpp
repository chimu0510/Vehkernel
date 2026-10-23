// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/core/timing_monitor.hpp>

#include <vk/msg/work_item_status.hpp>

#include <algorithm>
#include <cstdint>
#include <string_view>

namespace vk::core {

static_assert(max_work_items <= msg::WorkItemStatus::instances,
              "every work item needs its own work_item_status instance");

TimingMonitor::TimingMonitor(WorkQueue& queue, std::uint32_t interval_us) : WorkItem("timing_monitor", queue)
{
    schedule_periodic(interval_us);
}

void TimingMonitor::run()
{
    publish_all();
}

void TimingMonitor::publish_all()
{
    const std::uint64_t now = queue().now_us();
    for (const WorkItem* item : work_items()) {
        const WorkItemStats stats = item->stats();
        msg::WorkItemStatus status{};
        status.timestamp = now;
        status.work_item = static_cast<std::uint8_t>(item->index());
        const std::string_view name = item->name().substr(0, status.name.size());
        std::ranges::copy(name, status.name.begin());
        status.runs = stats.runs;
        status.overruns = stats.overruns;
        status.run_time_last_us = stats.run_time_last_us;
        status.run_time_max_us = stats.run_time_max_us;
        status.run_time_mean_us = stats.run_time_mean_us;
        status.jitter_max_us = stats.jitter_max_us;
        topic<msg::WorkItemStatus>(item->index()).publish(status);
    }
}

}  // namespace vk::core
