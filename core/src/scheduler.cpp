// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/core/scheduler.hpp>

#include <vk/core/static_instance.hpp>
#include <vk/core/timing_monitor.hpp>
#include <vk/core/work_queue.hpp>

#include <FreeRTOS.h>
#include <task.h>

namespace vk::core {
namespace {

StaticInstance<TimingMonitor> monitor;

}  // namespace

void start_scheduler()
{
    if (!monitor.has_value()) {
        monitor.emplace(work_queue(WorkQueueTier::Slow));
    }
    start_work_queues();
    vTaskStartScheduler();
}

void stop_scheduler()
{
    vTaskEndScheduler();
}

}  // namespace vk::core
