// SPDX-License-Identifier: GPL-3.0-or-later
// The remote control (spec 0010): what the gamepad's sticks and buttons ask
// of the vehicle. No bus, no FreeRTOS and no hardware, so it runs on the
// development host in tests; RemoteControlTask runs it as a work item.
//
// In MANUAL, with the remote control in control and connected, the sticks
// drive by body velocity: vx = MAN_VX_MAX · left y, yaw rate = MAN_YAW_MAX ·
// right x (right stick right turns right, z down), each after the deadzone
// MAN_DEADZONE, rescaled so the output starts from zero at its edge. vy stays
// 0 until a frame can move sideways.
//
// The buttons send requests, each once per press, measured in the
// controller's own cycles: A takes over in MANUAL from any mode, armed or
// not; B engages the emergency stop; START held for hold_us arms, BACK held
// for hold_us disarms; Y held for hold_us releases the emergency stop. Arming
// with a stick off centre is refused by the commander. Nothing is asked while
// the remote control is not connected.
#pragma once

#include <vk/core/manual_control.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/msg/enums.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace vk::remote_control {

using estimation::Twist;

// The MAN_* parameters.
struct Settings {
    float vx_max = 0.5F;     // MAN_VX_MAX [m/s]
    float yaw_max = 1.0F;    // MAN_YAW_MAX [rad/s]
    float deadzone = 0.08F;  // MAN_DEADZONE
};

// The remote control's input this cycle.
struct Input {
    bool connected = false;           // connected and heard within RC_TIMEOUT
    std::array<float, 4> sticks{};    // left x, left y, right x, right y
    std::uint16_t buttons = 0;        // bit n: msg::GamepadButton n
};

// One request to the commander.
struct Request {
    core::Command command{};
    std::uint8_t argument = 0;
};

// What one cycle asks.
struct Result {
    std::optional<Twist> setpoint;      // a body velocity setpoint to publish
    std::array<Request, 5> requests{};  // the first `request_count` are to send
    std::size_t request_count = 0;
};

class RemoteControl {
public:
    static constexpr std::uint64_t hold_us = 1'000'000;

    // One cycle at `now_us`, with the mode and the source in control from vehicle_status.
    Result step(std::uint64_t now_us, const Input& input, core::Mode mode, core::ControlSource in_control,
                const Settings& settings);

private:
    // The press state of one button.
    struct Button {
        std::uint64_t since_us = 0;
        bool pressed = false;
        bool fired = false;
    };

    // Whether `button` was pressed this cycle, or has now been held for hold_us.
    bool pressed_now(msg::GamepadButton button, const Input& input, std::uint64_t now_us);
    bool held(msg::GamepadButton button, const Input& input, std::uint64_t now_us);
    void track(msg::GamepadButton button, const Input& input, std::uint64_t now_us);

    std::array<Button, 16> buttons_{};
};

}  // namespace vk::remote_control
