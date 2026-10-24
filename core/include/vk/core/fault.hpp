// SPDX-License-Identifier: GPL-3.0-or-later
// The fault stop and the reset cause (ADR-0005).
//
// On a fault the platform turns every actuator output off by writing
// registers only, records why, and stops: on a board it halts for a debugger
// or waits for the hardware watchdog to reset it; in SITL it ends the run.
// Fault handlers, a failed configASSERT, fail() and the FreeRTOS stack
// overflow hook all end there. At boot the platform reads what caused the
// last reset, with the fault recorded before it, and the commander publishes
// it and refuses arming after a watchdog or fault reset.
#pragma once

#include <vk/msg/enums.hpp>

#include <cstdint>

namespace vk::core {

// Both are declared in the schema (schema/enum/, ADR-0006), as they go out
// on vehicle_status.
using ResetCause = msg::ResetCause;
using Fault = msg::Fault;

// What the platform read at boot.
struct ResetReport {
    ResetCause cause = ResetCause::Unknown;
    Fault fault = Fault::None;       // for a FAULT reset
    std::uint32_t fault_address = 0;  // the faulting instruction, or the caller; 0 if unknown

    friend bool operator==(const ResetReport&, const ResetReport&) = default;
};

// A reset nobody asked for: arming is refused after one until the next
// deliberate reset (a reboot request, the reset pin or a power cycle).
[[nodiscard]] constexpr bool unexpected(const ResetReport& report)
{
    return report.cause == ResetCause::Watchdog || report.cause == ResetCause::Fault;
}

// The fault stop: every actuator output off, the fault recorded, then the
// platform stops. Safe from any context, including a fault handler with
// interrupts masked. Defined by the platform layer, as the FreeRTOS hooks are.
[[noreturn]] void fault_stop(Fault fault, std::uint32_t address);

}  // namespace vk::core
