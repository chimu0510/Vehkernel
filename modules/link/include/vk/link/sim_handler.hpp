// SPDX-License-Identifier: GPL-3.0-or-later
// The SITL-only frames of a Link session (ADR-0018): sim_actuator_power cuts
// or restores the simulated actuator power, sim_user_key presses or releases
// the simulated user key, and sim_gamepad sets the simulated gamepad
// (spec 0010), none with a reply. sim_gamepad's body: connected (u8), the
// sticks left x, left y, right x, right y (i16 each, ±32767 full travel), the
// triggers left, right (u8 each, 255 full) and the buttons (u16, bit n:
// gamepad_button n). Only SITL gives a
// session this handler, and only SITL builds it, so the board firmware never
// accepts these frames.
#pragma once

#include <vk/link/frame.hpp>
#include <vk/link/frame_handler.hpp>

#include <array>
#include <cstdint>

namespace vk::link {

// The simulator a session may act on.
struct SimAccess {
    void (*set_actuator_power)(bool powered) = nullptr;
    void (*set_user_key)(bool pressed) = nullptr;
    void (*set_gamepad)(bool connected, const std::array<float, 4>& sticks, const std::array<float, 2>& triggers,
                        std::uint16_t buttons) = nullptr;
};

class SimHandler final : public FrameHandler {
public:
    explicit SimHandler(SimAccess access) : access_(access) {}

    // Handles sim_actuator_power, sim_user_key or sim_gamepad.
    void handle(FrameType type, BodyReader body, FrameSender& out) override;

private:
    SimAccess access_;
};

}  // namespace vk::link
