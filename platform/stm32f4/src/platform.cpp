// SPDX-License-Identifier: GPL-3.0-or-later
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

    // The cycle counter times short busy-waits such as software I2C bits.
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

}  // namespace vk::platform
