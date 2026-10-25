// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle description and the selection of its vehicle model at boot
// (ADR-0012, ADR-0013).
//
// VEHICLE_TYPE, FRAME_CLASS and FRAME_TYPE, the geometry and the output
// mapping are all reboot_required: they are read once, at boot, before
// anything that drives the outputs starts. FRAME_CLASS picks the vehicle
// model, which only models compiled in (Kconfig VEHICLE_MODEL_*) can be;
// a description no model can drive selects none, and nothing is allocated.
#pragma once

#include <vk/config.h>
#include <vk/msg/enums.hpp>
#include <vk/vehicle_model/vehicle_model.hpp>
#ifdef CONFIG_VEHICLE_MODEL_WHEELED
#include <vk/vehicle_model/wheeled_model.hpp>
#endif

namespace vk::vehicle_model {

struct VehicleDescription {
    msg::VehicleType vehicle_type = msg::VehicleType::Ground;  // VEHICLE_TYPE
    msg::FrameClass frame_class = msg::FrameClass::Wheeled;    // FRAME_CLASS
#ifdef CONFIG_VEHICLE_MODEL_WHEELED
    WheeledFrame wheeled{};  // FRAME_TYPE and the WHL_* parameters
#endif
};

// The description the parameters give.
VehicleDescription description_from_parameters();

// Holds one instance of every compiled-in vehicle model, so selecting one
// allocates nothing.
class VehicleModels {
public:
    // The model `description` selects, set up for it; nullptr when its frame
    // class is not compiled in or the description is invalid for it.
    const VehicleModel* select(const VehicleDescription& description);

private:
#ifdef CONFIG_VEHICLE_MODEL_WHEELED
    WheeledModel wheeled_;
#endif
};

}  // namespace vk::vehicle_model
