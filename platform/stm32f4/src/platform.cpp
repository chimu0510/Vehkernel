// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/platform/platform.hpp>

#include "internal.hpp"

#include <stm32f4xx.h>

namespace vk::platform {

void init()
{
    // Before anything else can clear the reset flags.
    detail::read_reset_report();
    // MemManage, BusFault and UsageFault get their own handlers instead of
    // escalating to HardFault, so the fault record tells them apart.
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk;
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
