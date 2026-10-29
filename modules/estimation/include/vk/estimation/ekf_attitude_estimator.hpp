// SPDX-License-Identifier: GPL-3.0-or-later
// The EKF attitude estimator (ADR-0014, spec 0012): an error-state Extended
// Kalman Filter as a work item on wq:ctrl, woken by each imu_sample publication
// (ADR-0008), draining magnetometer_sample each run.
//
// Each run drains every queued IMU sample and fuses queued magnetometer samples
// whose timestamps are not after the IMU sample just predicted to (ADR-0015).
// It first aligns: once the vehicle has been still for the alignment time, roll
// and pitch come from the mean accelerometer reading, the gyro bias from the mean
// gyro reading, and yaw from the mean magnetometer heading (with heading fusion on)
// or zero (with it off). The mean field strength becomes the reference for
// EKF_MAG_REJ. Movement restarts the wait. Nothing is published while aligning.
//
// Then every IMU sample predicts the filter and fuses gravity, and queued
// magnetometer samples within EKF_MAG_REJ of reference strength are fused as
// heading measurements. While the vehicle is at rest (a still IMU and a body
// velocity near zero), each 0.2 s average of the gyro is fused as a measurement
// of the gyro bias: the zero rate update, the only measurement of the z bias on
// level ground. Without heading fusion the yaw is relative to the start, so it
// starts with a variance near zero. Published on vehicle_attitude (when primary)
// or estimator_attitude (instance 1, when secondary).
//
// An estimator_restart returns it to aligning (ADR-0014). Samples (IMU,
// magnetometer and body velocity) taken before the restart are dropped, however
// late they reach the queue, so what the estimator does depends only on the
// timestamps a log records (ADR-0018).
//
// The filter noises, gates, declination and rejection thresholds are the EKF_*
// parameters (ADR-0012), read at construction and again after each parameter_update,
// so changes apply from the next sample.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/estimation/estimator_selection.hpp>
#include <vk/estimation/filters/attitude_eskf.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/rest_detector.hpp>
#include <vk/estimation/still_alignment.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/estimator_attitude.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/magnetometer_sample.hpp>
#include <vk/msg/parameter_update.hpp>
#include <vk/msg/vehicle_attitude.hpp>

#include <cstdint>
#include <optional>

namespace vk::estimation {

class EkfAttitudeEstimator final : public core::WorkItem {
public:
    static constexpr std::uint8_t estimator_id = 2;

    struct Config {
        std::uint8_t imu_instance = 0;           // imu_sample instance to read
        std::uint8_t mag_instance = 0;           // magnetometer_sample instance to read
        AttitudeEskfConfig filter{};
        StillAlignment::Config alignment{};
        RestDetector::Config rest_detector{};
        std::optional<bool> primary{};           // overrides EST_PRIMARY when set
        std::optional<EkfMagType> mag_type{};    // overrides EKF_MAG_TYPE / VEHICLE_TYPE when set
    };

    enum class State : std::uint8_t {
        Aligning,
        Running,
    };

    EkfAttitudeEstimator(const Config& config, core::WorkQueue& queue);

    // Listens to the IMU topic.
    void start();

    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] bool is_primary() const { return is_primary_; }
    [[nodiscard]] EkfMagType effective_mag_type() const { return effective_mag_type_; }
    [[nodiscard]] bool mag_missing() const { return mag_missing_; }
    [[nodiscard]] float reference_field_strength() const { return reference_field_strength_; }
    [[nodiscard]] const AttitudeEskf& filter() const { return filter_; }
    [[nodiscard]] const RestDetector& rest_detector() const { return rest_detector_; }

protected:
    void run() override;

private:
    void apply_parameters();
    // Returns the estimator to its state at boot. Samples taken before `at_us` no longer count.
    void restart(std::uint64_t at_us);
    void process_mag(const msg::MagnetometerSample& sample, bool record_in_alignment);
    void drain_mag_up_to(std::uint64_t timestamp_us, bool record_in_alignment);
    void drain_velocity_up_to(std::uint64_t timestamp_us);
    void align(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel);
    void estimate(const msg::ImuSample& sample, const Vector3& gyro, const Vector3& accel,
                  const std::optional<GyroAverage>& avg, std::uint64_t now);

    Config config_;
    core::Subscription<msg::ImuSample> imu_;
    core::Subscription<msg::MagnetometerSample> mag_;
    core::Subscription<msg::BodyVelocityMeasurement> velocity_{0};
    core::Subscription<msg::ParameterUpdate> parameter_update_;
    core::Subscription<msg::EstimatorRestart> restart_{0};
    core::Publication<msg::VehicleAttitude> attitude_{0};
    core::Publication<msg::EstimatorAttitude> secondary_attitude_{1};
    AttitudeEskf filter_;
    State state_ = State::Aligning;
    bool is_primary_ = false;
    EkfMagType effective_mag_type_ = EkfMagType::Off;
    bool mag_missing_ = false;
    float reference_field_strength_ = 0.0F;
    std::uint64_t restarted_at_us_ = 0;  // timestamp of the last estimator_restart

    StillAlignment alignment_;
    RestDetector rest_detector_;
    std::uint64_t aligned_at_us_ = 0;
    std::optional<msg::MagnetometerSample> pending_mag_{};
    std::optional<msg::BodyVelocityMeasurement> pending_velocity_{};
};

}  // namespace vk::estimation
