// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle model's body velocity measurement as a work item on wq:ctrl
// (ADR-0013, ADR-0014), run by each wheel_encoders sample.
//
// It takes the outputs' wheel speeds through the vehicle model selected at
// boot and publishes body_velocity_measurement on instance 0, source WHEELS.
// Each wheel's speed noise is its encoder's quantization: one count in a
// sample interval, as a uniform error, (2π / MOTn_CPR) / (interval·√12).
// Nothing is published without a model with wheel speed feedback, or while
// a wheel the model uses has no encoder.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/wheel_encoders.hpp>
#include <vk/vehicle_model/vehicle_model.hpp>

#include <cstdint>

namespace vk::vehicle_model {

class VelocityMeasurementTask final : public core::WorkItem {
public:
    struct Config {
        const VehicleModel* model = nullptr;  // the one selected at boot; nullptr for none
    };

    VelocityMeasurementTask(const Config& config, core::WorkQueue& queue);

    // Runs on every wheel_encoders sample from now on.
    void start();

protected:
    void run() override;

private:
    Config config_;
    core::Subscription<msg::WheelEncoders> encoders_{0};
    core::Publication<msg::BodyVelocityMeasurement> publication_{0};
    std::uint64_t last_sample_us_ = 0;
};

}  // namespace vk::vehicle_model
