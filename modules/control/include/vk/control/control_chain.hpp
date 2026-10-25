// SPDX-License-Identifier: GPL-3.0-or-later
// The control chain above actuator control (ADR-0013): a body velocity
// setpoint becomes a body motion demand, which the vehicle model allocates
// to a wheel speed target per actuator output. No bus, no FreeRTOS and no
// hardware, so it runs on the development host in tests; ControlChainTask
// runs it as a work item.
//
// Velocity control passes vx and vy to allocation as feedforward: the wheel
// speed loops of actuator control already close each wheel's speed. A
// setpoint with a NaN field, or no vehicle model, leaves every output
// uncontrolled.
#pragma once

#include <vk/vehicle_model/vehicle_model.hpp>

namespace vk::control {

using vehicle_model::OutputTargets;
using vehicle_model::Twist;
using vehicle_model::VehicleModel;
using vehicle_model::WheelAllocation;

class ControlChain {
public:
    // `model` may be nullptr: no vehicle model was selected at boot.
    explicit ControlChain(const VehicleModel* model) : model_(model) {}

    // One control cycle: the wheel speed targets for `setpoint`, none above
    // `max_speed` [rad/s] (MOT_MAX_SPD).
    WheelAllocation step(const Twist& setpoint, float max_speed, OutputTargets targets);

    [[nodiscard]] bool has_model() const { return model_ != nullptr; }

private:
    const VehicleModel* model_;
};

}  // namespace vk::control
