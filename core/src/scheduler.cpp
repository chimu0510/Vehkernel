#include <vk/core/scheduler.hpp>

#include <FreeRTOS.h>
#include <task.h>

namespace vk::core {

void start_scheduler()
{
    vTaskStartScheduler();
}

void stop_scheduler()
{
    vTaskEndScheduler();
}

}  // namespace vk::core
