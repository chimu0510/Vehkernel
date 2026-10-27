// SPDX-License-Identifier: GPL-3.0-or-later
// The local odometry estimator (ADR-0014): dead reckoning as a work item on
// wq:ctrl, woken by each body velocity measurement (ADR-0008).
//
// Each measurement on body_velocity_measurement instance `measurement`
// advances the position in the local level frame by its body velocity,
// turned by the heading of the newest vehicle_attitude: the attitude
// estimator's, never the wheels' speed difference, so a skid does not turn
// the track. It publishes vehicle_local_odometry with the attitude's
// quaternion and yaw rate and the measurement's velocity and variances.
// Nothing moves before the attitude estimator has aligned: the origin is
// where the vehicle stands then, and an attitude older than
// attitude_timeout_us holds the odometry where it is.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/estimation/models/dead_reckoning.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_local_odometry.hpp>

#include <cstdint>

namespace vk::estimation {

class LocalOdometry final : public core::WorkItem {
public:
    static constexpr std::uint8_t estimator_id = 1;
    static constexpr std::uint64_t attitude_timeout_us = 50'000;  // ten IMU samples at 200 Hz

    struct Config {
        std::uint8_t measurement = 0;  // body_velocity_measurement instance to read
    };

    LocalOdometry(const Config& config, core::WorkQueue& queue);

    // Listens to the measurement topic.
    void start();

    [[nodiscard]] const DeadReckoning& dead_reckoning() const { return dead_reckoning_; }

protected:
    void run() override;

private:
    Config config_;
    core::Subscription<msg::BodyVelocityMeasurement> measurement_;
    core::Subscription<msg::VehicleAttitude> attitude_{0};
    core::Subscription<msg::EstimatorRestart> restart_{0};
    core::Publication<msg::VehicleLocalOdometry> publication_;
    msg::VehicleAttitude attitude_message_{};
    DeadReckoning dead_reckoning_;
    std::uint64_t last_sample_us_ = 0;
};

}  // namespace vk::estimation
