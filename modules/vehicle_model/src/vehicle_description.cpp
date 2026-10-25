// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/vehicle_model/vehicle_description.hpp>

#include <vk/params/params.hpp>

#include <cstdint>

namespace vk::vehicle_model {

VehicleDescription description_from_parameters()
{
    VehicleDescription description;
    description.vehicle_type = static_cast<msg::VehicleType>(params::vehicle.type);
    description.frame_class = static_cast<msg::FrameClass>(params::frame.frame_class);
#ifdef CONFIG_VEHICLE_MODEL_WHEELED
    const auto& whl = params::whl;
    description.wheeled = {
        .type = static_cast<msg::FrameType>(params::frame.type),
        .radius = whl.radius,
        .track = whl.track,
        .outputs = {static_cast<std::uint8_t>(whl.left_out), static_cast<std::uint8_t>(whl.right_out),
                    static_cast<std::uint8_t>(whl.rear_l_out), static_cast<std::uint8_t>(whl.rear_r_out)},
    };
#endif
    return description;
}

const VehicleModel* VehicleModels::select(const VehicleDescription& description)
{
    if (description.vehicle_type != msg::VehicleType::Ground) {
        return nullptr;
    }
    switch (description.frame_class) {
#ifdef CONFIG_VEHICLE_MODEL_WHEELED
    case msg::FrameClass::Wheeled:
        wheeled_.set(description.wheeled);
        return wheeled_.valid() ? &wheeled_ : nullptr;
#endif
    default:
        return nullptr;
    }
}

}  // namespace vk::vehicle_model
