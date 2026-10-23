// SPDX-License-Identifier: GPL-3.0-or-later
// Microsecond clock on TIM7 (ADR-0008).
//
// TIM7 is a basic timer no board function can use, so the board definition
// never assigns it. It counts at 1 MHz and its overflow interrupt, at a
// priority FreeRTOS never masks, counts the 16-bit wraps.
#include "internal.hpp"

#include <vk/hal/clock.hpp>
#include <vk/platform/counter_extension.hpp>

#include <board.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_tim.h>

#include <cstdint>

namespace {

constexpr std::uint32_t counter_hz = 1'000'000;
static_assert(vk::board::clock::apb1_timer_hz % counter_hz == 0,
              "the APB1 timer clock must divide down to exactly 1 MHz");

// Above every interrupt that may call FreeRTOS (configMAX_SYSCALL_INTERRUPT_PRIORITY).
constexpr std::uint32_t overflow_irq_priority = 1;

volatile std::uint64_t overflows = 0;

}  // namespace

extern "C" void TIM7_IRQHandler()
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U) {
        LL_TIM_ClearFlag_UPDATE(TIM7);
        overflows = overflows + 1;
    }
}

namespace vk::platform::detail {

void start_microsecond_clock()
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
    // Stop the clock while a debugger halts the core, as SysTick does, so a
    // halt does not show up as run time, jitter or overruns.
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP;
    LL_TIM_SetPrescaler(TIM7, (vk::board::clock::apb1_timer_hz / counter_hz) - 1U);
    LL_TIM_SetAutoReload(TIM7, 0xFFFFU);
    LL_TIM_GenerateEvent_UPDATE(TIM7);  // Load the prescaler now.
    LL_TIM_ClearFlag_UPDATE(TIM7);
    LL_TIM_EnableIT_UPDATE(TIM7);
    NVIC_SetPriority(TIM7_IRQn, overflow_irq_priority);
    NVIC_EnableIRQ(TIM7_IRQn);
    LL_TIM_EnableCounter(TIM7);
}

}  // namespace vk::platform::detail

std::uint64_t vk::hal::SystemClock::now_us() noexcept
{
    // The 64-bit overflow count is two loads, so the reading is taken with
    // interrupts masked; extend_counter() accounts for a wrap not yet counted.
    const std::uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const std::uint64_t count = overflows;
    const auto counter = static_cast<std::uint16_t>(LL_TIM_GetCounter(TIM7));
    const bool pending = LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U;
    __set_PRIMASK(primask);
    return vk::platform::extend_counter(count, counter, pending);
}
