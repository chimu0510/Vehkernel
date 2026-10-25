// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/estimation/local_odometry.hpp>

#include <vk/estimation/math/quaternion.hpp>

namespace vk::estimation {

LocalOdometry::LocalOdometry(const Config& config, core::WorkQueue& queue)
    : WorkItem("local_odometry", queue), config_(config), measurement_(config.measurement)
{
}

void LocalOdometry::start()
{
    trigger_on(core::topic<msg::BodyVelocityMeasurement>(config_.measurement));
}

void LocalOdometry::run()
{
    msg::BodyVelocityMeasurement measurement{};
    while (measurement_.copy(measurement)) {
        while (attitude_.copy(attitude_message_)) {
            // Only the newest counts.
        }
        const std::uint64_t sample = measurement.timestamp_sample;
        const bool heading = attitude_message_.timestamp != 0 && sample <= attitude_message_.timestamp + attitude_timeout_us &&
                             attitude_message_.timestamp <= sample + attitude_timeout_us;
        if (!heading) {
            // No heading yet, or a stale one: the odometry waits where it is.
            last_sample_us_ = sample;
            continue;
        }
        const float dt = last_sample_us_ == 0 || sample <= last_sample_us_
                             ? 0.0F
                             : static_cast<float>(sample - last_sample_us_) * 1e-6F;
        last_sample_us_ = sample;
        const Quaternion q{attitude_message_.q[0], attitude_message_.q[1], attitude_message_.q[2],
                           attitude_message_.q[3]};
        dead_reckoning_.step(dt, measurement.vx, measurement.vy, measurement.variance[0], measurement.variance[1],
                             q.euler().yaw, attitude_message_.variance[2]);

        msg::VehicleLocalOdometry out{};
        out.timestamp = queue().now_us();
        out.timestamp_sample = sample;
        out.estimator_id = estimator_id;
        out.position = {static_cast<float>(dead_reckoning_.north()), static_cast<float>(dead_reckoning_.east()), 0.0F};
        out.q = attitude_message_.q;
        out.velocity = {measurement.vx, measurement.vy, 0.0F};
        out.yaw_rate = attitude_message_.angular_velocity[2];
        out.position_variance = dead_reckoning_.covariance();
        out.heading_variance = attitude_message_.variance[2];
        out.velocity_variance = measurement.variance;
        publication_.publish(out);
    }
}

}  // namespace vk::estimation
