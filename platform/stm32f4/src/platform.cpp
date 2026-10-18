#include <vk/platform/platform.hpp>

#include "internal.hpp"

#include <stm32f4xx.h>

namespace vk::platform {

void init()
{
    // FreeRTOS on Cortex-M needs every priority bit as preemption priority.
    NVIC_SetPriorityGrouping(0);
    detail::configure_clock_tree();
    detail::start_microsecond_clock();
}

}  // namespace vk::platform
