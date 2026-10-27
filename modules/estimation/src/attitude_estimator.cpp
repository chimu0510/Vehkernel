// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/estimation/attitude_estimator.hpp>

#include <vk/params/params.hpp>

#include <cmath>

namespace vk::estimation {

AttitudeEstimator::AttitudeEstimator(const Config& config, core::WorkQueue& queue)
    : WorkItem("mahony_attitude", queue), config_(config), imu_(config.imu_instance),
      filter_(config.filter)
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
    apply_parameters();
}

void AttitudeEstimator::apply_parameters()
{
    filter_.set_gains(params::att.kp, params::att.ki, params::att.acc_rej);
}

void AttitudeEstimator::restart()
{
    state_ = State::Aligning;
    still_since_us_ = 0;
    still_samples_ = 0;
    gyro_sum_ = {};
    accel_sum_ = {};
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
        restart();
    }
    msg::ImuSample sample{};
    while (imu_.copy(sample)) {
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
    const float g = config_.filter.gravity;
    const bool still = gyro.norm() < config_.still_gyro && std::abs(accel.norm() - g) < config_.still_accel * g;
    if (!still) {
        still_samples_ = 0;
        return;
    }
    if (still_samples_ == 0) {
        still_since_us_ = sample.timestamp;
        gyro_sum_ = {};
        accel_sum_ = {};
    }
    ++still_samples_;
    gyro_sum_ += gyro;
    accel_sum_ += accel;
    if (sample.timestamp - still_since_us_ < config_.alignment_us) {
        return;
    }
    const auto n = static_cast<float>(still_samples_);
    filter_.reset_from(accel_sum_ * (1.0F / n), gyro_sum_ * (1.0F / n));
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
    attitude_.publish(out);
}

}  // namespace vk::estimation
