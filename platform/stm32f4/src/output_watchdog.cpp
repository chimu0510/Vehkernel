// SPDX-License-Identifier: GPL-3.0-or-later
// The output watchdog on TIM6 (ADR-0005, spec 0006).
//
// TIM6 is a basic timer no board function can use. It interrupts every
// millisecond at a priority FreeRTOS never masks, so it runs whatever the
// work queues do; when actuator control has not refreshed the outputs for
// three control periods it stops every output through the callback given at
// start. The interrupt writes registers only.
//
// The fault stop (fault.cpp) runs above this interrupt, or with it masked, so
// it turns the outputs off itself, through the same callback.
#include <vk/hal/clock.hpp>
#include <vk/hal/output_watchdog.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/pwm_timing.hpp>

#include "internal.hpp"

#include <board.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_tim.h>

#include <cstdint>
#include <optional>

namespace {

constexpr std::uint32_t check_hz = 1'000;
// Above every interrupt that may call FreeRTOS, as the microsecond clock's.
constexpr std::uint32_t check_irq_priority = 2;

std::optional<vk::hal::OutputWatchdog> watchdog;
void (*stop_outputs)() = nullptr;

extern "C" std::uint32_t __vk_flash_firmware_start[];  // NOLINT: linker symbols
extern "C" std::uint32_t __vk_flash_firmware_end[];    // NOLINT

// Whether `stop` points into the firmware image: after a fault RAM may be corrupt.
bool in_firmware(void (*stop)())
{
    const auto address = reinterpret_cast<std::uintptr_t>(stop);
    return address >= reinterpret_cast<std::uintptr_t>(__vk_flash_firmware_start) &&
           address < reinterpret_cast<std::uintptr_t>(__vk_flash_firmware_end);
}

}  // namespace

extern "C" void TIM6_DAC_IRQHandler()
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) == 0U) {
        return;
    }
    LL_TIM_ClearFlag_UPDATE(TIM6);
    if (watchdog && stop_outputs != nullptr && watchdog->expired(vk::hal::SystemClock::now_us())) {
        stop_outputs();
    }
}

namespace vk::platform {

void refresh_output_watchdog()
{
    if (watchdog) {
        watchdog->refresh(hal::SystemClock::now_us());
    }
}

void start_output_watchdog(std::uint32_t period_us, void (*stop)())
{
    watchdog.emplace(period_us);
    stop_outputs = stop;
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
    // Stops with the core in a debugger, like the clock it reads.
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM6_STOP;
    LL_TIM_SetPrescaler(TIM6, (vk::board::clock::apb1_timer_hz / 1'000'000U) - 1U);
    LL_TIM_SetAutoReload(TIM6, (1'000'000U / check_hz) - 1U);
    LL_TIM_GenerateEvent_UPDATE(TIM6);
    LL_TIM_ClearFlag_UPDATE(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);
    NVIC_SetPriority(TIM6_DAC_IRQn, check_irq_priority);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    LL_TIM_EnableCounter(TIM6);
}

std::uint32_t output_watchdog_trips()
{
    return watchdog ? watchdog->trips() : 0;
}

void detail::force_outputs_off()
{
    // MOE cleared: with OSSI set and the idle levels low, every channel of the
    // timer drives low at once (ADR-0005). Writes to a timer whose clock is
    // off are ignored, so this holds before set_frequency() too.
    TIM1->BDTR &= ~advanced_timer_moe;
    TIM8->BDTR &= ~advanced_timer_moe;
    // No duty and the direction lines low.
    void (*const stop)() = stop_outputs;
    if (stop != nullptr && in_firmware(stop)) {
        stop();
    }
}

}  // namespace vk::platform
