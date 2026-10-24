// SPDX-License-Identifier: GPL-3.0-or-later
// The reset cause on STM32F4 (ADR-0005), free of hardware so unit tests run
// it on the host.
//
// RCC_CSR's reset flags (RM0090, section 7.3.21) say what reset the MCU; the
// fault stop's record in RTC backup registers 1 and 2 says that a reset
// followed a fault, and which. The flags add up until cleared, so the
// platform clears them, and the record, at every boot. The MCU drives NRST
// low for every internal reset, so the pin flag counts only alone. The
// record counts only with a watchdog or software reset, so one left behind
// by a fault that a power cycle ended cannot outlive the next boot.
#pragma once

#include <vk/core/fault.hpp>
#include <vk/msg/enums.hpp>

#include <cstdint>

namespace vk::platform {

// RCC_CSR's reset flags.
namespace reset_flags {
inline constexpr std::uint32_t brownout = 1U << 25U;   // BORRSTF
inline constexpr std::uint32_t pin = 1U << 26U;        // PINRSTF
inline constexpr std::uint32_t power_on = 1U << 27U;   // PORRSTF
inline constexpr std::uint32_t software = 1U << 28U;   // SFTRSTF
inline constexpr std::uint32_t iwdg = 1U << 29U;       // IWDGRSTF
inline constexpr std::uint32_t wwdg = 1U << 30U;       // WWDGRSTF
inline constexpr std::uint32_t low_power = 1U << 31U;  // LPWRRSTF
}  // namespace reset_flags

// The fault stop's record in backup register 1: this marker with the fault
// in the low byte. Backup register 2 holds its address.
inline constexpr std::uint32_t fault_record_marker = 0xFA017000U;
inline constexpr std::uint32_t fault_record_mask = 0xFFFFFF00U;

constexpr std::uint32_t fault_record(core::Fault fault)
{
    return fault_record_marker | static_cast<std::uint8_t>(fault);
}

// The fault a record holds; NONE for anything but a fault the schema names.
constexpr core::Fault recorded_fault(std::uint32_t record)
{
    if ((record & fault_record_mask) != fault_record_marker) {
        return core::Fault::None;
    }
    for (const auto& entry : msg::EnumValues<core::Fault>::values) {
        if (static_cast<std::uint8_t>(entry.value) == (record & ~fault_record_mask)) {
            return entry.value;
        }
    }
    return core::Fault::None;
}

// What reset the MCU, from RCC_CSR and the fault record read at boot.
constexpr core::ResetReport decode_reset(std::uint32_t csr, std::uint32_t record, std::uint32_t address)
{
    using core::ResetCause;
    const core::Fault fault = recorded_fault(record);
    const bool recorded = fault != core::Fault::None;
    if ((csr & reset_flags::low_power) != 0U) {
        return {.cause = ResetCause::LowPower};
    }
    if ((csr & (reset_flags::iwdg | reset_flags::wwdg | reset_flags::software)) != 0U) {
        if (recorded) {
            return {.cause = ResetCause::Fault, .fault = fault, .fault_address = address};
        }
        const bool watchdog = (csr & (reset_flags::iwdg | reset_flags::wwdg)) != 0U;
        return {.cause = watchdog ? ResetCause::Watchdog : ResetCause::Software};
    }
    if ((csr & (reset_flags::power_on | reset_flags::brownout)) != 0U) {
        return {.cause = ResetCause::PowerOn};
    }
    if ((csr & reset_flags::pin) != 0U) {
        return {.cause = ResetCause::Pin};
    }
    return {};
}

}  // namespace vk::platform
