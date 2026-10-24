// SPDX-License-Identifier: GPL-3.0-or-later
// FreeRTOS application hooks for STM32F4 (ADR-0004, ADR-0008). The stack
// overflow hook is the fault stop's (fault.cpp, ADR-0005).
//
// Task stacks go into CCM, leaving normal SRAM for DMA (ADR-0008).
#include <vk/platform/platform.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <stm32f4xx.h>

#include <array>

namespace {

StaticTask_t idle_tcb;
[[gnu::section(".ccm")]] std::array<StackType_t, configMINIMAL_STACK_SIZE> idle_stack;

}  // namespace

extern "C" {

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb, StackType_t** stack, configSTACK_DEPTH_TYPE* stack_size)
{
    *tcb = &idle_tcb;
    *stack = idle_stack.data();
    *stack_size = idle_stack.size();
}

// Refresh the hardware watchdog from the lowest priority there is, so a
// scheduler that never gets idle resets the board (ADR-0005), then sleep
// until the next interrupt. DBGMCU keeps the debug clocks running, so SWD can
// still attach (see startup_stm32f407.c).
void vApplicationIdleHook()
{
    vk::platform::refresh_hardware_watchdog();
    __WFI();
}

}  // extern "C"
