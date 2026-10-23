// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle control service (ADR-0005): mode, arming, emergency stop and
// reboot (ADR-0023) requests. Like the parameter runtime (ADR-0012) it is protocol
// independent, so the custom protocol now and MAVLink and the Bridge later
// reach the commander through the same interface.
//
// A source is whoever sends a request: a Link instance today, the remote
// control and the Bridge later. Requests are idempotent, so a tool may repeat
// one it got no reply to.
#pragma once

#include <vk/msg/enums.hpp>

#include <cstdint>

namespace vk::core {

// The commander's enums are part of the wire protocol, so they are declared
// in the schema (schema/enum/, ADR-0006) and generated with their names.
using Mode = msg::Mode;                  // the mode decides who is in control
using ModeResult = msg::ModeResult;      // why a mode request was refused
using ArmingResult = msg::ArmingResult;  // why an arm request was refused
using RebootAction = msg::RebootAction;  // numbered as MAVLink's (ADR-0023)
using RebootResult = msg::RebootResult;  // why a reboot request was refused
using DisarmReason = msg::DisarmReason;  // why the vehicle last disarmed
using BringupCase = msg::BringupCase;    // bring-up cases run in test mode (ADR-0018)
using BringupStart = msg::BringupStart;  // why a bring-up request was refused

// A Link instance, or later the remote control or the Bridge.
using ControlSource = std::uint8_t;
inline constexpr ControlSource no_source = 0xFF;

// Bits of COM_ARM_CHECK: checks of hardware state that can be skipped.
namespace arm_checks {
inline constexpr std::uint32_t actuator_power = 1U << 0U;
inline constexpr std::uint32_t all = actuator_power;
}  // namespace arm_checks

class VehicleControl {
public:
    VehicleControl() = default;
    virtual ~VehicleControl() = default;
    VehicleControl(const VehicleControl&) = delete;
    VehicleControl& operator=(const VehicleControl&) = delete;
    VehicleControl(VehicleControl&&) = delete;
    VehicleControl& operator=(VehicleControl&&) = delete;

    // Changes the mode; entering TEST makes `source` the source in control.
    virtual ModeResult request_mode(ControlSource source, Mode mode) = 0;

    // Arms if every arming check passes; disarming is accepted from any source.
    virtual ArmingResult request_arming(ControlSource source, bool arm) = 0;

    // Engages or releases the emergency stop, from any source; returns whether it is engaged.
    virtual bool request_emergency_stop(bool engage) = 0;

    [[nodiscard]] virtual Mode mode() const = 0;
    [[nodiscard]] virtual bool armed() const = 0;

    // Bits of arm_checks currently skipped.
    [[nodiscard]] virtual std::uint32_t skipped_checks() const = 0;

    // Setpoint arbitration (ADR-0005): whether a direct-actuator setpoint from
    // `source` is accepted now: from the source in control, in TEST, while
    // armed and no bring-up case runs.
    [[nodiscard]] virtual bool accepts_setpoint(ControlSource source) const = 0;

    // Starts a bring-up case, from the source in control, in TEST, while armed.
    virtual BringupStart request_bringup(ControlSource source, std::uint8_t bringup_case) = 0;

    // `source` fell silent: a bring-up case it runs is aborted and the vehicle disarmed.
    virtual void source_lost(ControlSource source) = 0;

    // Reboots the board, or reboots it into the bootloader, from any source,
    // while disarmed. The reply goes out before the board resets.
    virtual RebootResult request_reboot(RebootAction action) = 0;
};

}  // namespace vk::core
