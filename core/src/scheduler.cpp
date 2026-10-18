#include <vk/core/scheduler.hpp>

#include <vk/core/timing_monitor.hpp>
#include <vk/core/work_queue.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <cstddef>
#include <new>

namespace vk::core {
namespace {

// Created once and never destroyed, so no exit-time destructor is registered.
alignas(TimingMonitor) std::byte monitor_storage[sizeof(TimingMonitor)];  // NOLINT(modernize-avoid-c-arrays)
TimingMonitor* monitor = nullptr;

}  // namespace

void start_scheduler()
{
    if (monitor == nullptr) {
        monitor = new (monitor_storage) TimingMonitor(work_queue(WorkQueueTier::Slow));
    }
    start_work_queues();
    vTaskStartScheduler();
}

void stop_scheduler()
{
    vTaskEndScheduler();
}

}  // namespace vk::core
