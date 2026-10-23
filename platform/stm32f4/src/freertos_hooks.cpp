// SPDX-License-Identifier: GPL-3.0-or-later
// FreeRTOS application hooks for STM32F4 (ADR-0004, ADR-0008).
//
// Task stacks go into CCM, leaving normal SRAM for DMA (ADR-0008).
#include <FreeRTOS.h>
#include <task.h>

#include <stm32f4xx.h>

#include <array>

namespace {

StaticTask_t idle_tcb;
[[gnu::section(".ccm")]] std::array<StackType_t, configMINIMAL_STACK_SIZE> idle_stack;

[[noreturn]] void halt()
{
    __disable_irq();
    for (;;) {
    }
}

}  // namespace

extern "C" {

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb, StackType_t** stack, configSTACK_DEPTH_TYPE* stack_size)
{
    *tcb = &idle_tcb;
    *stack = idle_stack.data();
    *stack_size = idle_stack.size();
}

// Sleep until the next interrupt. DBGMCU keeps the debug clocks running, so
// SWD can still attach (see startup_stm32f407.c).
void vApplicationIdleHook()
{
    __WFI();
}

void vApplicationStackOverflowHook(TaskHandle_t /*task*/, char* /*name*/)
{
    halt();
}

}  // extern "C"
