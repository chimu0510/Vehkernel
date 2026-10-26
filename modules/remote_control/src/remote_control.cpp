// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/remote_control/remote_control.hpp>

namespace vk::remote_control {

void RemoteControl::track(msg::GamepadButton button, const Input& input, std::uint64_t now_us)
{
    Button& b = buttons_.at(static_cast<std::size_t>(button));
    const bool down = input.connected && (input.buttons & (1U << static_cast<unsigned>(button))) != 0;
    if (down && !b.pressed) {
        b.since_us = now_us;
        b.fired = false;
    }
    b.pressed = down;
}

bool RemoteControl::pressed_now(msg::GamepadButton button, const Input& input, std::uint64_t now_us)
{
    track(button, input, now_us);
    Button& b = buttons_.at(static_cast<std::size_t>(button));
    if (b.pressed && !b.fired) {
        b.fired = true;
        return true;
    }
    return false;
}

bool RemoteControl::held(msg::GamepadButton button, const Input& input, std::uint64_t now_us)
{
    track(button, input, now_us);
    Button& b = buttons_.at(static_cast<std::size_t>(button));
    if (b.pressed && !b.fired && now_us - b.since_us >= hold_us) {
        b.fired = true;
        return true;
    }
    return false;
}

Result RemoteControl::step(std::uint64_t now_us, const Input& input, core::Mode mode, core::ControlSource in_control,
                           const Settings& settings)
{
    using msg::GamepadButton;
    Result result;
    const auto ask = [&result](core::Command command, std::uint8_t argument) {
        result.requests.at(result.request_count++) = {.command = command, .argument = argument};
    };
    // Every button is tracked every cycle, so a press made while another
    // fires is not missed.
    const bool take_over = pressed_now(GamepadButton::A, input, now_us);
    const bool stop = pressed_now(GamepadButton::B, input, now_us);
    const bool release = held(GamepadButton::Y, input, now_us);
    const bool arm = held(GamepadButton::Start, input, now_us);
    const bool disarm = held(GamepadButton::Back, input, now_us);
    if (!input.connected) {
        return result;
    }
    // The emergency stop first, so it is never behind another request.
    if (stop) {
        ask(core::Command::EmergencyStop, 1);
    }
    if (release) {
        ask(core::Command::EmergencyStop, 0);
    }
    if (take_over) {
        ask(core::Command::Mode, static_cast<std::uint8_t>(core::Mode::Manual));
    }
    if (disarm) {
        ask(core::Command::Arm, 0);
    }
    if (arm) {
        ask(core::Command::Arm, 1);
    }
    if (mode == core::Mode::Manual && in_control == core::remote_control) {
        result.setpoint = Twist{.vx = settings.vx_max * core::apply_deadzone(input.sticks[1], settings.deadzone),
                                .vy = 0.0F,
                                .yaw_rate = settings.yaw_max * core::apply_deadzone(input.sticks[2], settings.deadzone)};
    }
    return result;
}

}  // namespace vk::remote_control
