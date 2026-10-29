// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/estimation/ekf_attitude_estimator.hpp>
#include <vk/estimation/estimator_selection.hpp>

#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/estimation/models/magnetometer_model.hpp>
#include <vk/params/params.hpp>

#include <cmath>
#include <numbers>

namespace vk::estimation {

namespace {
StillAlignment::Config make_alignment_config(const EkfAttitudeEstimator::Config& config)
{
    StillAlignment::Config cfg = config.alignment;
    cfg.gravity = config.filter.gravity;
    return cfg;
}

RestDetector::Config make_rest_detector_config(const EkfAttitudeEstimator::Config& config)
{
    RestDetector::Config cfg = config.rest_detector;
    cfg.imu = make_alignment_config(config);
    return cfg;
}
}  // namespace

EkfAttitudeEstimator::EkfAttitudeEstimator(const Config& config, core::WorkQueue& queue)
    : WorkItem("ekf_attitude", queue), config_(config), imu_(config.imu_instance),
      mag_(config.mag_instance), velocity_(0), filter_(config.filter),
      alignment_(make_alignment_config(config)), rest_detector_(make_rest_detector_config(config))
{
    // Drop stale messages already queued.
    msg::ImuSample stale_imu{};
    while (imu_.copy(stale_imu)) {
    }
    msg::MagnetometerSample stale_mag{};
    while (mag_.copy(stale_mag)) {
    }
    msg::BodyVelocityMeasurement stale_vel{};
    while (velocity_.copy(stale_vel)) {
    }
    msg::ParameterUpdate seen{};
    parameter_update_.copy(seen);
    msg::EstimatorRestart stale_restart{};
    while (restart_.copy(stale_restart)) {
    }

    // Determine whether this estimator publishes vehicle_attitude (primary).
    if (config_.primary.has_value()) {
        is_primary_ = *config_.primary;
    } else {
        is_primary_ = (primary_estimator() == PrimaryEstimator::Ekf);
    }

    // Determine effective magnetometer heading fusion mode.
    if (config_.mag_type.has_value()) {
        effective_mag_type_ = *config_.mag_type;
    } else {
#ifdef CONFIG_ESTIMATOR_EKF
        const auto mag_type = static_cast<EkfMagType>(params::ekf.mag_type);
#else
        const auto mag_type = EkfMagType::Auto;
#endif
        const auto vehicle_type = static_cast<msg::VehicleType>(params::vehicle.type);
        effective_mag_type_ = resolve_mag_type(mag_type, vehicle_type);
    }

    apply_parameters();
}

void EkfAttitudeEstimator::apply_parameters()
{
    AttitudeEskfConfig cfg = filter_.config();
#ifdef CONFIG_ESTIMATOR_EKF
    cfg.gyro_noise = params::ekf.gyr_noise;
    cfg.gyro_bias_noise = params::ekf.gyr_b_noise;
    cfg.accel_noise = params::ekf.acc_noise;
    cfg.accel_rejection = params::ekf.acc_rej;
    cfg.accel_gate = params::ekf.acc_gate;
    cfg.mag_noise = params::ekf.mag_noise;
    cfg.mag_gate = params::ekf.mag_gate;
    cfg.mag_declination = params::ekf.mag_decl * (std::numbers::pi_v<float> / 180.0F);
    cfg.mag_rejection = params::ekf.mag_rej;
#endif
    filter_.set_config(cfg);
}

void EkfAttitudeEstimator::restart(std::uint64_t at_us)
{
    state_ = State::Aligning;
    restarted_at_us_ = at_us;
    alignment_.reset();
    rest_detector_.reset();
    aligned_at_us_ = 0;
    pending_mag_.reset();
    pending_velocity_.reset();
    mag_missing_ = false;
    reference_field_strength_ = 0.0F;
    filter_ = AttitudeEskf(config_.filter);
    apply_parameters();
}

void EkfAttitudeEstimator::start()
{
    trigger_on(core::topic<msg::ImuSample>(config_.imu_instance));
}

void EkfAttitudeEstimator::process_mag(const msg::MagnetometerSample& sample, bool record_in_alignment)
{
    const Vector3 field = Vector3::from(sample.field);
    if (record_in_alignment) {
        alignment_.feed_mag(field);
    } else if (effective_mag_type_ == EkfMagType::Heading && !mag_missing_) {
        filter_.fuse_heading(field, reference_field_strength_);
    }
}

void EkfAttitudeEstimator::drain_mag_up_to(std::uint64_t timestamp_us, bool record_in_alignment)
{
    // First process any pending magnetometer sample held from a previous step.
    if (pending_mag_.has_value()) {
        if (pending_mag_->timestamp < restarted_at_us_) {
            pending_mag_.reset();
        } else if (pending_mag_->timestamp <= timestamp_us) {
            process_mag(*pending_mag_, record_in_alignment);
            pending_mag_.reset();
        }
    }

    // Now drain incoming magnetometer samples.
    msg::MagnetometerSample sample{};
    while (!pending_mag_.has_value() && mag_.copy(sample)) {
        if (sample.timestamp < restarted_at_us_) {
            continue;
        }
        if (sample.timestamp <= timestamp_us) {
            process_mag(sample, record_in_alignment);
        } else {
            // Later sample: keep it for the next IMU sample.
            pending_mag_ = sample;
            break;
        }
    }
}

void EkfAttitudeEstimator::drain_velocity_up_to(std::uint64_t timestamp_us)
{
    if (pending_velocity_.has_value()) {
        if (pending_velocity_->timestamp < restarted_at_us_) {
            pending_velocity_.reset();
        } else if (pending_velocity_->timestamp <= timestamp_us) {
            rest_detector_.feed_velocity(pending_velocity_->timestamp, pending_velocity_->vx,
                                         pending_velocity_->vy, pending_velocity_->yaw_rate);
            pending_velocity_.reset();
        }
    }

    msg::BodyVelocityMeasurement sample{};
    while (!pending_velocity_.has_value() && velocity_.copy(sample)) {
        if (sample.timestamp < restarted_at_us_) {
            continue;
        }
        if (sample.timestamp <= timestamp_us) {
            rest_detector_.feed_velocity(sample.timestamp, sample.vx, sample.vy, sample.yaw_rate);
        } else {
            pending_velocity_ = sample;
            break;
        }
    }
}

void EkfAttitudeEstimator::run()
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
            continue;
        }
        const Vector3 gyro = Vector3::from(sample.gyro);
        const Vector3 accel = Vector3::from(sample.accel);
        drain_velocity_up_to(sample.timestamp);
        const auto avg = rest_detector_.feed_imu(sample.timestamp, gyro, accel);
        if (state_ == State::Aligning) {
            drain_mag_up_to(sample.timestamp, true);
            align(sample, gyro, accel);
        } else {
            estimate(sample, gyro, accel, avg, queue().now_us());
        }
    }
}

void EkfAttitudeEstimator::align(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel)
{
    if (!alignment_.feed_imu(sample.timestamp, gyro, accel)) {
        return;
    }

    const Vector3 mean_acc = alignment_.mean_accel();
    const Vector3 mean_gyr = alignment_.mean_gyro();
    float initial_yaw = 0.0F;
    mag_missing_ = false;
    reference_field_strength_ = 0.0F;

    if (effective_mag_type_ == EkfMagType::Heading) {
        if (alignment_.has_mag()) {
            const Euler level = AccelerometerModel::level_from(mean_acc);
            initial_yaw = MagnetometerModel::heading_from(level.roll, level.pitch, alignment_.mean_mag(),
                                                         filter_.config().mag_declination);
            reference_field_strength_ = alignment_.mean_mag_strength();
        } else {
            // Heading fusion on with no magnetometer sample in the alignment second:
            // yaw at zero, no magnetometer fusion, and a warning on vehicle_status.
            mag_missing_ = true;
            initial_yaw = 0.0F;
            reference_field_strength_ = 0.0F;
        }
    }

    std::optional<float> initial_yaw_variance{};
    if (effective_mag_type_ != EkfMagType::Heading || mag_missing_) {
        initial_yaw_variance = 1e-6F;
    }

    filter_.reset_from(mean_acc, mean_gyr, initial_yaw, initial_yaw_variance);
    filter_.predict(gyro, sample.timestamp);  // sets the filter's time
    aligned_at_us_ = sample.timestamp;
    state_ = State::Running;
}

void EkfAttitudeEstimator::estimate(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel,
                                    const std::optional<GyroAverage>& avg, std::uint64_t now)
{
    filter_.predict(gyro, sample.timestamp);
    filter_.fuse_gravity(accel);
    drain_mag_up_to(sample.timestamp, false);

    if (avg.has_value()) {
        const float gyr_noise = filter_.config().gyro_noise;
        const float variance = (gyr_noise * gyr_noise) / avg->duration_seconds;
        filter_.fuse_zero_rate(avg->mean_gyro, variance);
    }

    msg::VehicleAttitude out{};
    out.timestamp = now;
    out.timestamp_sample = sample.timestamp;
    out.estimator_id = estimator_id;
    out.q = filter_.attitude().array();
    out.angular_velocity = filter_.angular_velocity().array();
    out.gyro_bias = filter_.gyro_bias().array();
    out.variance = filter_.attitude_variance().array();
    out.accel_rejected = filter_.accel_rejected();
    out.mag_rejected = filter_.mag_rejected();
    out.mag_fused = filter_.mag_fused();
    out.zero_rate_fused = filter_.zero_rate_fused();
    out.mag_missing = mag_missing_;

    publish_attitude(is_primary_, out, attitude_, secondary_attitude_);
}

}  // namespace vk::estimation
