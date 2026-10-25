// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/sim_handler.hpp>

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
    default:
        break;
    }
}

}  // namespace vk::link
