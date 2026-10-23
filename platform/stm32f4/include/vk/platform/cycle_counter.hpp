// SPDX-License-Identifier: GPL-3.0-or-later
// Busy-wait delays from the Cortex-M4 cycle counter (DWT CYCCNT).
//
// vk::platform::init() starts the counter. Only for delays far shorter than
// a tick, such as bit timing; anything longer waits on the scheduler.
#pragma once

#include <stm32f4xx.h>

#include <cstdint>

namespace vk::platform {

inline std::uint32_t cycles()
{
    return DWT->CYCCNT;
}

inline void delay_cycles(std::uint32_t count)
{
    const std::uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < count) {
    }
}

}  // namespace vk::platform
