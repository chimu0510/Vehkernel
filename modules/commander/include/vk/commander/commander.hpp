// The commander (ADR-0005): modes, arming, arming checks and the emergency
// stop, as a state machine with no bus, no FreeRTOS and no hardware, so it
// runs on the development host in tests. CommanderTask runs it as a work item.
//
// The vehicle boots in STANDBY. A mode changes only while disarmed; entering
// TEST makes the requesting source the source in control. An arm request
// passes the arming checks in the order of ArmingResult; the checks of
// hardware state (only actuator power so far) are skipped by clearing their
// bit in COM_ARM_CHECK.
//
// A setpoint counts while it is current: accepted since arming when armed,
// or within COM_SP_TIMEOUT when disarmed, so one left from before a disarm
// neither blocks arming for long nor drives anything after it.
//
// A bring-up case is aborted, by disarming, when the emergency stop is
// engaged or its source falls silent while it runs.
//
// update() runs once per control cycle with that cycle's inputs. It tracks
// actuator power, on boards that sense it: one unpowered sample is
// unpowered and disarms an armed vehicle unless the check is skipped;
// powered counts only after unbroken powered samples for the settle time.
// It also disarms an armed vehicle when the emergency stop has been engaged
// for 5 s, when the setpoints from the source in control stop for longer
// than COM_SP_TIMEOUT after the first one, and when every target has been
// idle for COM_DISARM_IDLE. Nothing ever re-arms on its own.
#pragma once

#include <vk/core/vehicle_control.hpp>

#include <cstdint>
#include <optional>

namespace vk::commander {

// The commander's parameters (COM_*), as the work item reads them.
struct Settings {
    std::uint32_t arm_check = core::arm_checks::all;  // COM_ARM_CHECK
    float setpoint_timeout_s = 0.5F;                   // COM_SP_TIMEOUT
    float disarm_idle_s = 10.0F;                       // COM_DISARM_IDLE
};

// What one control cycle observed.
struct Inputs {
    bool board_verified = false;                // the IMU answered and the revision matched (ADR-0010)
    std::optional<bool> actuator_power;         // the sample read powered; none if the board cannot sense it
    bool actuators_started = false;
    bool setpoints_idle = true;                 // every target of the last setpoint NaN or zero
    bool bringup_running = false;
    bool bringup_available = false;             // the bring-up case is built in
    bool local_menu_open = false;               // the local menu is open (ADR-0022)
    std::uint64_t last_setpoint_us = 0;         // when the last setpoint was accepted, 0 for never
    std::uint32_t output_watchdog_trips = 0;    // times the platform's output watchdog turned the outputs off
};

enum class ActuatorPower : std::uint8_t {
    NotAvailable = 0,
    Unpowered = 1,
    Settling = 2,
    Powered = 3,
};

struct State {
    core::Mode mode = core::Mode::Standby;
    bool armed = false;
    core::ControlSource source = core::no_source;
    bool board_verified = false;
    bool emergency_stop = false;
    ActuatorPower actuator_power = ActuatorPower::NotAvailable;
    bool actuator_power_raw = false;
    std::uint32_t skipped_checks = 0;
    core::ArmingResult arming_refusal = core::ArmingResult::Ok;
    core::DisarmReason disarm_reason = core::DisarmReason::None;
    std::uint32_t output_watchdog_trips = 0;

    friend bool operator==(const State&, const State&) = default;
};

class Commander {
public:
    // Unbroken powered samples needed before actuator power counts as present.
    static constexpr std::uint64_t power_settle_us = 500'000;
    // How long the emergency stop may stay engaged before the vehicle disarms.
    static constexpr std::uint64_t emergency_stop_disarm_us = 5'000'000;

    // Once per control cycle.
    void update(std::uint64_t now_us, const Inputs& inputs, const Settings& settings);

    core::ModeResult request_mode(std::uint64_t now_us, core::ControlSource source, core::Mode mode);
    core::ArmingResult request_arming(std::uint64_t now_us, core::ControlSource source, bool arm);
    bool request_emergency_stop(std::uint64_t now_us, bool engage);

    [[nodiscard]] core::BringupStart request_bringup(core::ControlSource source, std::uint8_t bringup_case) const;

    // A source fell silent; aborts a bring-up case it runs by disarming.
    void source_lost(core::ControlSource source);

    // Setpoint arbitration: whether a setpoint from `source` is accepted now.
    [[nodiscard]] bool accepts_setpoint(core::ControlSource source) const;

    [[nodiscard]] const State& state() const { return state_; }

private:
    [[nodiscard]] core::ArmingResult arming_check(std::uint64_t now_us, core::ControlSource source) const;
    [[nodiscard]] bool power_checked() const;
    void track_power(std::uint64_t now_us, std::optional<bool> sample);
    void arm(std::uint64_t now_us);
    void disarm(core::DisarmReason reason);

    State state_{};
    Inputs inputs_{};
    Settings settings_{};
    std::uint64_t power_since_us_ = 0;      // first of the unbroken powered samples
    std::uint64_t engaged_at_us_ = 0;       // when the emergency stop was engaged
    std::uint64_t armed_at_us_ = 0;
    std::uint64_t active_at_us_ = 0;        // last time a target was set or a bring-up case ran
    std::uint64_t timeout_from_us_ = 0;     // the setpoint timeout counts from here
};

}  // namespace vk::commander
