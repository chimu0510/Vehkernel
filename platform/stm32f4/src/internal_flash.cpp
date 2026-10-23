// SPDX-License-Identifier: GPL-3.0-or-later
// The STM32F4 Flash controller (RM0090 section 3): sector erase and 32-bit
// programming with PSIZE x32, which needs a supply of 2.7 V to 3.6 V, as on
// the C50C.
#include <vk/platform/internal_flash.hpp>

#include <stm32f4xx.h>

namespace vk::platform::detail {
namespace {

constexpr std::uint32_t errors =
    FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_SOP;

void wait_idle()
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0) {
    }
}

void unlock()
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0) {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
    FLASH->SR = errors | FLASH_SR_EOP;  // clear what an earlier operation left
}

void lock()
{
    FLASH->CR |= FLASH_CR_LOCK;
}

}  // namespace

bool flash_erase_sector(std::uint8_t sector)
{
    wait_idle();
    unlock();
    FLASH->CR = (FLASH->CR & ~(FLASH_CR_PSIZE | FLASH_CR_SNB)) | FLASH_CR_PSIZE_1 |
                (static_cast<std::uint32_t>(sector) << FLASH_CR_SNB_Pos) | FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;
    wait_idle();
    const bool ok = (FLASH->SR & errors) == 0;
    FLASH->CR &= ~(FLASH_CR_SER | FLASH_CR_SNB);
    lock();
    // The data cache may still hold the old contents; it can only be reset while off.
    if ((FLASH->ACR & FLASH_ACR_DCEN) != 0) {
        FLASH->ACR &= ~FLASH_ACR_DCEN;
        FLASH->ACR |= FLASH_ACR_DCRST;
        FLASH->ACR &= ~FLASH_ACR_DCRST;
        FLASH->ACR |= FLASH_ACR_DCEN;
    }
    return ok;
}

hal::FlashResult flash_program_word(std::uint32_t address, std::uint32_t word)
{
    auto* target = reinterpret_cast<volatile std::uint32_t*>(address);  // NOLINT(performance-no-int-to-ptr)
    if (*target != 0xFFFFFFFFU) {
        return hal::FlashResult::NotErased;
    }
    wait_idle();
    unlock();
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PSIZE) | FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    *target = word;
    __DSB();
    wait_idle();
    const bool ok = (FLASH->SR & errors) == 0 && *target == word;
    FLASH->CR &= ~FLASH_CR_PG;
    lock();
    return ok ? hal::FlashResult::Ok : hal::FlashResult::Error;
}

}  // namespace vk::platform::detail
