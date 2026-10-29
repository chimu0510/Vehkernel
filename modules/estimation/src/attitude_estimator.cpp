// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/estimation/attitude_estimator.hpp>
#include <vk/estimation/estimator_selection.hpp>

#include <vk/params/params.hpp>

#include <cmath>

namespace vk::estimation {

namespace {
StillAlignment::Config make_alignment_config(const AttitudeEstimator::Config& config)
{
    StillAlignment::Config cfg = config.alignment;
    cfg.gravity = config.filter.gravity;
    return cfg;
}
}  // namespace

AttitudeEstimator::AttitudeEstimator(const Config& config, core::WorkQueue& queue)
    : WorkItem("mahony_attitude", queue), config_(config), imu_(config.imu_instance),
      filter_(config.filter), alignment_(make_alignment_config(config))
{
    // Only samples published from now on count.
    msg::ImuSample stale{};
    while (imu_.copy(stale)) {
    }
    msg::ParameterUpdate seen{};
    parameter_update_.copy(seen);
    msg::EstimatorRestart stale_restart{};
    while (restart_.copy(stale_restart)) {
    }

    if (config_.primary.has_value()) {
        is_primary_ = *config_.primary;
    } else {
        is_primary_ = (primary_estimator() == PrimaryEstimator::Mahony);
    }

    apply_parameters();
}

void AttitudeEstimator::apply_parameters()
{
    filter_.set_gains(params::att.kp, params::att.ki, params::att.acc_rej);
}

void AttitudeEstimator::restart(std::uint64_t at_us)
{
    state_ = State::Aligning;
    restarted_at_us_ = at_us;
    alignment_.reset();
    aligned_at_us_ = 0;
    filter_ = MahonyFilter(config_.filter);
    apply_parameters();
}

void AttitudeEstimator::start()
{
    trigger_on(core::topic<msg::ImuSample>(config_.imu_instance));
}

void AttitudeEstimator::run()
{
    msg::ParameterUpdate update{};
    if (parameter_update_.copy(update)) {
        apply_parameters();
    }
    msg::EstimatorRestart restart_msg{};
    while (restart_.copy(restart_msg)) {
        restart(restart_msg.timestamp);
    }
    msg::ImuSample sample{};
    while (imu_.copy(sample)) {
        if (sample.timestamp < restarted_at_us_) {
            // Taken before the restart, though it reaches the queue after it: the log orders
            // inputs by these timestamps, and so does the replay (ADR-0018).
            continue;
        }
        const Vector3 gyro = Vector3::from(sample.gyro);
        const Vector3 accel = Vector3::from(sample.accel);
        if (state_ == State::Aligning) {
            align(sample, gyro, accel);
        } else {
            estimate(sample, gyro, accel, queue().now_us());
        }
    }
}

void AttitudeEstimator::align(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel)
{
    if (!alignment_.feed_imu(sample.timestamp, gyro, accel)) {
        return;
    }
    filter_.reset_from(alignment_.mean_accel(), alignment_.mean_gyro());
    filter_.update(gyro, accel, sample.timestamp);  // sets the filter's time
    aligned_at_us_ = sample.timestamp;
    state_ = State::Running;
}

void AttitudeEstimator::estimate(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel,
                                 std::uint64_t now)
{
    filter_.update(gyro, accel, sample.timestamp);
    const float since_alignment_s = static_cast<float>(sample.timestamp - aligned_at_us_) * 1e-6F;

    msg::VehicleAttitude out{};
    out.timestamp = now;
    out.timestamp_sample = sample.timestamp;
    out.estimator_id = estimator_id;
    out.q = filter_.attitude().array();
    out.angular_velocity = filter_.angular_velocity().array();
    out.gyro_bias = filter_.gyro_bias().array();
    out.variance = {config_.level_variance, config_.level_variance, config_.yaw_variance_rate * since_alignment_s};
    out.accel_rejected = filter_.accel_rejected();
    out.mag_rejected = 0;
    out.mag_fused = 0;
    out.zero_rate_fused = 0;
    out.mag_missing = false;

    publish_attitude(is_primary_, out, attitude_, secondary_attitude_);
}

}  // namespace vk::estimation
