// SPDX-License-Identifier: GPL-3.0-or-later
// The SITL-only frames of a Link session (ADR-0018): sim_actuator_power cuts
// or restores the simulated actuator power and sim_user_key presses or
// releases the simulated user key, neither with a reply. Only SITL gives a
// session this handler, and only SITL builds it, so the board firmware never
// accepts these frames.
#pragma once

#include <vk/link/frame.hpp>
#include <vk/link/frame_handler.hpp>

namespace vk::link {

// The simulator a session may act on.
struct SimAccess {
    void (*set_actuator_power)(bool powered) = nullptr;
    void (*set_user_key)(bool pressed) = nullptr;
};

class SimHandler final : public FrameHandler {
public:
    explicit SimHandler(SimAccess access) : access_(access) {}

    // Handles sim_actuator_power or sim_user_key.
    void handle(FrameType type, BodyReader body, FrameSender& out) override;

private:
    SimAccess access_;
};

}  // namespace vk::link
