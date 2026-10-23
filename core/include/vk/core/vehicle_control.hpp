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

#include <cstdint>

namespace vk::core {

enum class Mode : std::uint8_t {
    Standby = 0,  // no source in control; nothing can be armed
    Test = 1,     // the development host in control, direct-actuator setpoints only (ADR-0018)
};

// A Link instance, or later the remote control or the Bridge.
using ControlSource = std::uint8_t;
inline constexpr ControlSource no_source = 0xFF;

enum class ModeResult : std::uint8_t {
    Ok = 0,
    Armed = 1,        // the mode changes only while disarmed
    UnknownMode = 2,
};

// Why an arm request was refused, in the order the checks run.
enum class ArmingResult : std::uint8_t {
    Ok = 0,
    BoardNotVerified = 1,     // ADR-0010
    EmergencyStop = 2,        // engaged
    WrongMode = 3,            // only TEST can be armed for now
    NotInControl = 4,         // the request did not come from the source in control
    SetpointsActive = 5,      // a setpoint is neither NaN nor zero
    BringupRunning = 6,
    ActuatorPowerOff = 7,     // unpowered or still settling, and the check is not skipped
    ActuatorsNotStarted = 8,
    LocalMenuOpen = 9,        // ADR-0022
    Rebooting = 10,           // a reboot was accepted and is about to happen
};

// Reboot actions, numbered as MAVLink's MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
// param1, so MAVLink maps onto them directly (ADR-0023).
enum class RebootAction : std::uint8_t {
    Reboot = 1,      // start the firmware again
    Bootloader = 3,  // stay in the bootloader until flashed or reset
};

// Why a reboot request was refused.
enum class RebootResult : std::uint8_t {
    Ok = 0,
    Armed = 1,          // a reboot happens only while disarmed (ADR-0005)
    UnknownAction = 2,
    NotAvailable = 3,   // the platform cannot reset
};

// Why the vehicle last disarmed.
enum class DisarmReason : std::uint8_t {
    None = 0,
    Request = 1,
    EmergencyStop = 2,       // engaged for 5 s while armed
    ActuatorPowerLost = 3,
    SetpointTimeout = 4,
    Idle = 5,
    BringupAbort = 6,
};

// Bring-up cases run in test mode (ADR-0018).
enum class BringupCase : std::uint8_t {
    Motors = 1,  // motor-to-encoder mapping and direction
};

// Why a bring-up request was refused, in the order the checks run.
enum class BringupStart : std::uint8_t {
    Ok = 0,
    NotAvailable = 1,   // not built in
    UnknownCase = 2,
    WrongMode = 3,
    NotArmed = 4,
    NotInControl = 5,
    EmergencyStop = 6,
    Running = 7,        // a case is already running
};

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
