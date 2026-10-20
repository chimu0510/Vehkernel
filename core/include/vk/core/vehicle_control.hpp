// The vehicle control service (ADR-0005): mode, arming and emergency stop
// requests. Like the parameter runtime (ADR-0012) it is protocol
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
};

}  // namespace vk::core
