// SPDX-License-Identifier: GPL-3.0-or-later
// The fault stop, the hardware watchdog and the reset cause on STM32F4
// (ADR-0005).
//
// HardFault, MemManage, BusFault and UsageFault, any exception or interrupt
// without a handler (startup_stm32f407.c), a failed configASSERT, fail() and
// the stack overflow hook all end in the fault stop, as Betaflight's fault
// handlers end in systemFaultAction(). It masks interrupts, turns every
// actuator output off by writing registers only, records the fault in RTC
// backup registers 1 and 2, then stops: at a breakpoint if a debugger is
// attached, otherwise in a loop until the IWDG resets the board. The IWDG is
// started there if it was not running yet, so a fault always ends in a reset.
//
// The IWDG, as ArduPilot's stm32_watchdog_init(), times out after about 2 s
// (1.4 s to 3.9 s over the LSI's 17 to 47 kHz) and is refreshed from the
// FreeRTOS idle hook, the lowest priority there is: a scheduler that never
// gets idle, as with a task or an interrupt that never returns, resets the
// board. It stops while a debugger halts the core (DBGMCU), and a reset stops
// it, so the bootloader runs without it.
#include <vk/core/fault.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/reset_cause.hpp>

#include "internal.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <stm32f4xx.h>

#include <cstddef>
#include <cstdint>

static_assert(vk::platform::reset_flags::brownout == RCC_CSR_BORRSTF);
static_assert(vk::platform::reset_flags::pin == RCC_CSR_PINRSTF);
static_assert(vk::platform::reset_flags::power_on == RCC_CSR_PORRSTF);
static_assert(vk::platform::reset_flags::software == RCC_CSR_SFTRSTF);
static_assert(vk::platform::reset_flags::iwdg == RCC_CSR_IWDGRSTF);
static_assert(vk::platform::reset_flags::wwdg == RCC_CSR_WWDGRSTF);
static_assert(vk::platform::reset_flags::low_power == RCC_CSR_LPWRRSTF);

// The fault numbers the handlers below load, as the schema declares them.
static_assert(static_cast<int>(vk::core::Fault::HardFault) == 1);
static_assert(static_cast<int>(vk::core::Fault::MemManage) == 2);
static_assert(static_cast<int>(vk::core::Fault::BusFault) == 3);
static_assert(static_cast<int>(vk::core::Fault::UsageFault) == 4);
static_assert(static_cast<int>(vk::core::Fault::UnexpectedInterrupt) == 8);

extern "C" {
extern std::uint32_t _estack[];         // NOLINT: linker symbols
extern std::uint32_t __vk_ram_start[];  // NOLINT
extern std::uint32_t __vk_ccm_start[];  // NOLINT
extern std::uint32_t __vk_ccm_end[];    // NOLINT

// For a debugger: the exception frame the fault stacked, and the fault.
[[gnu::used]] const std::uint32_t* volatile vk_fault_frame = nullptr;
[[gnu::used]] volatile std::uint8_t vk_fault = 0;
}

namespace {

constexpr std::uint32_t iwdg_key_unlock = 0x5555U;
constexpr std::uint32_t iwdg_key_start = 0xCCCCU;
constexpr std::uint32_t iwdg_key_refresh = 0xAAAAU;
constexpr std::uint32_t iwdg_prescaler = 3U;  // LSI / 32: 1 kHz from a 32 kHz LSI
constexpr std::uint32_t iwdg_reload = 2047U;   // 2.048 s at 1 kHz

// The words of an exception frame: r0 to r3, r12, lr, pc and xPSR.
constexpr std::uintptr_t frame_bytes = 8 * sizeof(std::uint32_t);
constexpr std::size_t frame_pc = 6;

vk::core::ResetReport report{};
volatile bool stopping = false;

bool in(std::uintptr_t address, const std::uint32_t* start, const std::uint32_t* end)
{
    return address >= reinterpret_cast<std::uintptr_t>(start) &&
           address + frame_bytes <= reinterpret_cast<std::uintptr_t>(end);
}

// The stacked pc, if the frame lies in RAM: after a stack overflow it may not,
// and reading it would fault inside the fault handler.
std::uint32_t stacked_pc(const std::uint32_t* frame)
{
    const auto address = reinterpret_cast<std::uintptr_t>(frame);
    if ((address % sizeof(std::uint32_t)) != 0U ||
        !(in(address, __vk_ram_start, _estack) || in(address, __vk_ccm_start, __vk_ccm_end))) {
        return 0;
    }
    return frame[frame_pc];  // NOLINT: an exception frame
}

}  // namespace

namespace vk::core {

void fault_stop(Fault fault, std::uint32_t address)
{
    __disable_irq();
    // The outputs first, then the record. A second fault on the way, in a
    // stop callback for example, comes back here and goes straight to the end.
    const bool again = stopping;
    stopping = true;
    if (!again) {
        platform::detail::force_outputs_off();
        vk_fault = static_cast<std::uint8_t>(fault);
        platform::detail::unlock_backup_registers();
        RTC->BKP1R = platform::fault_record(fault);
        RTC->BKP2R = address;
    }
    // The IWDG resets the board unless a debugger halts the core. Started here
    // in case it was not running yet, with the timeout it was given or its
    // reset value, 0.5 s.
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = iwdg_key_start;
    if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U) {
        __BKPT(0);
    }
    for (;;) {
    }
}

}  // namespace vk::core

extern "C" {

// Every fault handler arrives here with the fault in r0 and the EXC_RETURN in
// lr. The stack may have overflowed, so the main stack restarts from its top
// before any C++ runs; nothing returns, so nothing on it is needed again.
[[gnu::naked, gnu::used]] void vk_fault_trampoline()
{
    __asm__ volatile(
        "tst lr, #4\n\t"
        "ite eq\n\t"
        "mrseq r1, msp\n\t"
        "mrsne r1, psp\n\t"
        "ldr r2, =_estack\n\t"
        "msr msp, r2\n\t"
        "b vk_fault_entry\n\t"
        ".ltorg");
}

[[noreturn, gnu::used]] void vk_fault_entry(std::uint32_t fault, const std::uint32_t* frame)
{
    // The outputs before anything that may fault again, such as reading the frame.
    __disable_irq();
    vk::platform::detail::force_outputs_off();
    vk_fault_frame = frame;
    vk::core::fault_stop(static_cast<vk::core::Fault>(fault), stacked_pc(frame));
}

[[gnu::naked]] void HardFault_Handler()
{
    __asm__ volatile("movs r0, #1\n\tb vk_fault_trampoline");
}

[[gnu::naked]] void MemManage_Handler()
{
    __asm__ volatile("movs r0, #2\n\tb vk_fault_trampoline");
}

[[gnu::naked]] void BusFault_Handler()
{
    __asm__ volatile("movs r0, #3\n\tb vk_fault_trampoline");
}

[[gnu::naked]] void UsageFault_Handler()
{
    __asm__ volatile("movs r0, #4\n\tb vk_fault_trampoline");
}

// configASSERT (FreeRTOSConfig.h), with the address of the failed assertion.
void vk_assert_failed()
{
    vk::core::fault_stop(vk::core::Fault::Assert,
                         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(__builtin_return_address(0))));
}

void vApplicationStackOverflowHook(TaskHandle_t /*task*/, char* /*name*/)
{
    vk::core::fault_stop(vk::core::Fault::StackOverflow, 0);
}

}  // extern "C"

namespace vk::platform {

void detail::read_reset_report()
{
    const std::uint32_t csr = RCC->CSR;
    report = decode_reset(csr, RTC->BKP1R, RTC->BKP2R);
    RCC->CSR |= RCC_CSR_RMVF;
    unlock_backup_registers();
    RTC->BKP1R = 0;
    RTC->BKP2R = 0;
}

core::ResetReport reset_report()
{
    return report;
}

void start_hardware_watchdog()
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = iwdg_key_unlock;
    IWDG->PR = iwdg_prescaler;
    IWDG->RLR = iwdg_reload;
    IWDG->KR = iwdg_key_start;
    IWDG->KR = iwdg_key_refresh;
}

void refresh_hardware_watchdog()
{
    IWDG->KR = iwdg_key_refresh;
}

}  // namespace vk::platform
