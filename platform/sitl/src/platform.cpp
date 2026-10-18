#include <vk/platform/platform.hpp>

#include <vk/core/scheduler.hpp>
#include <vk/hal/clock.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstdint>

namespace vk::platform {
namespace {

StaticTask_t stop_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> stop_stack;
std::uint64_t stop_duration_us = 0;

void stop_task(void* /*unused*/)
{
    // Highest priority, so the run ends on time whatever else is busy.
    const std::uint64_t ticks = stop_duration_us * configTICK_RATE_HZ / 1'000'000U;
    vTaskDelay(static_cast<TickType_t>(ticks));
    core::stop_scheduler();
}

}  // namespace

void init()
{
    // The first reading fixes the clock's zero at boot.
    static_cast<void>(hal::SystemClock::now_us());
}

void stop_after(std::uint64_t duration_us)
{
    stop_duration_us = duration_us;
    xTaskCreateStatic(stop_task, "sitl_stop", stop_stack.size(), nullptr, configMAX_PRIORITIES - 1,
                      stop_stack.data(), &stop_tcb);
}

}  // namespace vk::platform
