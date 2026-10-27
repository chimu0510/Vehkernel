// SPDX-License-Identifier: GPL-3.0-or-later
// Vehicle control (ADR-0005): the vocabulary of requests to the commander and
// of setpoint arbitration, shared by every source and every consumer.
//
// A source is whoever sends a request or a setpoint: a Link instance, the
// remote control, and later the Bridge. Requests travel on the bus, on
// vehicle_command, and the commander answers each on vehicle_command_ack;
// they are idempotent, so a source may repeat one it got no answer to.
// Setpoints are tagged with their source, and every consumer drops those the
// mode does not accept from it, by the rule below, judged on the newest
// vehicle_status. The commander applies the same rule and counts the drops.
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
using Command = msg::Command;            // what a vehicle_command asks
using RestartEstimatorsResult = msg::RestartEstimatorsResult;  // why a restart_estimators request was refused

// A Link instance, or later the remote control or the Bridge.
using ControlSource = std::uint8_t;
inline constexpr ControlSource no_source = 0xFF;

// Bits of COM_ARM_CHECK: checks of hardware state that can be skipped.
namespace arm_checks {
inline constexpr std::uint32_t actuator_power = 1U << 0U;
inline constexpr std::uint32_t all = actuator_power;
}  // namespace arm_checks

// The remote control's source: Links are numbered from 0 below it.
inline constexpr ControlSource remote_control = 0x10;

constexpr bool is_link(ControlSource source)
{
    return source < remote_control;
}

// The setpoint levels of ADR-0003 built so far.
enum class SetpointLevel : std::uint8_t {
    DirectActuator,
    BodyVelocity,
};

// What a mode allows (ADR-0005): whether it can be armed and entered while
// armed, which sources may request it (and so be in control in it), the
// setpoint levels it accepts, and whether arming in it needs the remote
// control connected with its sticks centred.
struct ModeRules {
    bool armable = false;
    bool enter_while_armed = false;
    bool from_link = false;
    bool from_remote_control = false;
    bool direct_actuator = false;
    bool body_velocity = false;
    bool needs_remote_control = false;
};

constexpr bool known_mode(std::uint8_t mode)
{
    return mode <= static_cast<std::uint8_t>(Mode::Manual);
}

constexpr ModeRules mode_rules(Mode mode)
{
    switch (mode) {
    case Mode::Standby:
        return {.from_link = true, .from_remote_control = true};
    case Mode::Test:
        return {.armable = true, .from_link = true, .direct_actuator = true, .body_velocity = true};
    case Mode::Manual:
        return {.armable = true,
                .enter_while_armed = true,
                .from_remote_control = true,
                .body_velocity = true,
                .needs_remote_control = true};
    }
    return {};
}

// Whether `source` may request `mode`.
constexpr bool may_request(Mode mode, ControlSource source)
{
    const ModeRules rules = mode_rules(mode);
    return is_link(source) ? rules.from_link : source == remote_control && rules.from_remote_control;
}

// Whether `mode` accepts setpoints at `level`.
constexpr bool mode_accepts(Mode mode, SetpointLevel level)
{
    const ModeRules rules = mode_rules(mode);
    return level == SetpointLevel::DirectActuator ? rules.direct_actuator : rules.body_velocity;
}

// Setpoint arbitration (ADR-0005): whether a setpoint at `level` from
// `source` counts, in `mode`, with `in_control` the source in control.
// Whether it then drives anything is up to arming and the emergency stop.
constexpr bool setpoint_accepted(Mode mode, ControlSource in_control, ControlSource source, SetpointLevel level)
{
    return source != no_source && source == in_control && mode_accepts(mode, level);
}

}  // namespace vk::core
