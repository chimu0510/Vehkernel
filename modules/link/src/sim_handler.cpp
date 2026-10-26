// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/sim_handler.hpp>

#include <array>
#include <cstdint>

namespace vk::link {

void SimHandler::handle(FrameType type, BodyReader body, FrameSender& /*out*/)
{
    const std::uint8_t value = body.u8();
    if (!body.ok()) {
        return;
    }
    switch (type) {
    case FrameType::SimActuatorPower:
        if (access_.set_actuator_power != nullptr) {
            access_.set_actuator_power(value != 0);
        }
        break;
    case FrameType::SimUserKey:
        if (access_.set_user_key != nullptr) {
            access_.set_user_key(value != 0);
        }
        break;
    case FrameType::SimGamepad: {
        std::array<float, 4> sticks{};
        for (float& stick : sticks) {
            stick = static_cast<float>(static_cast<std::int16_t>(body.u16())) / 32767.0F;
        }
        std::array<float, 2> triggers{};
        for (float& trigger : triggers) {
            trigger = static_cast<float>(body.u8()) / 255.0F;
        }
        const std::uint16_t buttons = body.u16();
        if (body.ok() && access_.set_gamepad != nullptr) {
            access_.set_gamepad(value != 0, sticks, triggers, buttons);
        }
        break;
    }
    default:
        break;
    }
}

}  // namespace vk::link
