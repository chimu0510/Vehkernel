// SPDX-License-Identifier: GPL-3.0-or-later
// Restarting the board (ADR-0023). The request for the bootloader goes into
// RTC backup register 0, where both ArduPilot's and PX4's bootloaders read
// it on the STM32F4, with the value of the bootloader the board has:
// ArduPilot's RTC_BOOT_HOLD (stay) and RTC_BOOT_FAST (start the firmware at
// once), which the Vehkernel bootloader shares, or PX4's BOOT_RTC_SIGNATURE.
// The register survives the reset; the bootloader clears it.
#include <vk/platform/platform.hpp>

#include "internal.hpp"

#include <board.hpp>

#include <stm32f4xx.h>

#include <cstdint>

namespace vk::platform {

namespace {

constexpr std::uint32_t ardupilot_boot_hold = 0xb0070001;  // RTC_BOOT_HOLD
constexpr std::uint32_t ardupilot_boot_fast = 0xb0070002;  // RTC_BOOT_FAST
constexpr std::uint32_t px4_boot_signature = 0xb007b007;   // BOOT_RTC_SIGNATURE

std::uint32_t boot_value(hal::BootTarget target)
{
    using Kind = vk::board::bootloader::Kind;
    if constexpr (vk::board::bootloader::kind == Kind::Px4) {
        return target == hal::BootTarget::Bootloader ? px4_boot_signature : 0;
    } else {
        return target == hal::BootTarget::Bootloader ? ardupilot_boot_hold : ardupilot_boot_fast;
    }
}

}  // namespace

// As ArduPilot's set_rtc_backup: the backup domain is write-protected after
// reset, and the RTC is clocked from the LSI if nothing clocks it yet.
void detail::unlock_backup_registers()
{
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    if ((RCC->BDCR & RCC_BDCR_RTCEN) == 0U) {
        RCC->BDCR |= RCC_BDCR_RTCSEL_1;  // LSI
        RCC->BDCR |= RCC_BDCR_RTCEN;
    }
}

void reset(hal::BootTarget target)
{
    __disable_irq();
    detail::unlock_backup_registers();
    RTC->BKP0R = boot_value(target);
    NVIC_SystemReset();
}

}  // namespace vk::platform
