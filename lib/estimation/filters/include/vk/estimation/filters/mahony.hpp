// SPDX-License-Identifier: GPL-3.0-or-later
// Mahony complementary filter (ADR-0014): attitude from the gyro, with roll
// and pitch corrected from the accelerometer.
//
// The gyro propagates the attitude through quaternion kinematics. The
// accelerometer model predicts the up direction in the body frame; its cross
// product with the measured direction is the attitude error. Kp feeds the
// error back into the rate, and Ki integrates it into an estimate of the gyro
// bias. A specific force far from gravity means the vehicle is accelerating,
// so the correction is skipped and counted. Yaw is not observed: it follows
// the bias-corrected gyro.
#pragma once

#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/estimation/models/quaternion_kinematics.hpp>

#include <cmath>
#include <cstdint>

namespace vk::estimation {

struct MahonyConfig {
    float kp = 1.0F;               // [1/s] attitude correction gain
    float ki = 0.05F;              // [1/s^2] gyro bias gain
    float accel_rejection = 0.1F;  // skip the correction when | |a| - g | exceeds this fraction of g
    float gravity = standard_gravity;
    float max_dt = 0.1F;           // [s] longer gaps between samples are cut to this
};

class MahonyFilter {
public:
    using Config = MahonyConfig;

    explicit MahonyFilter(const Config& config = {}) : config_(config) {}

    // Starts from an attitude and a gyro bias; the next sample only sets the time.
    void reset(const Quaternion& attitude, const Vector3& gyro_bias = {})
    {
        q_ = attitude.normalized();
        bias_ = gyro_bias;
        have_timestamp_ = false;
    }

    // Starts level by a specific force measured at rest, with yaw 0.
    void reset_from(const Vector3& specific_force, const Vector3& gyro_bias = {})
    {
        reset(Quaternion::from_euler(AccelerometerModel::level_from(specific_force)), gyro_bias);
    }

    // One IMU sample: body rate [rad/s], specific force [m/s^2], and the time it was taken [us].
    void update(const Vector3& gyro, const Vector3& specific_force, std::uint64_t timestamp_us)
    {
        if (!have_timestamp_ || timestamp_us <= last_timestamp_us_) {
            // The first sample, or one out of order, only sets the time.
            if (!have_timestamp_ || timestamp_us < last_timestamp_us_) {
                last_timestamp_us_ = timestamp_us;
            }
            have_timestamp_ = true;
            return;
        }
        float dt = static_cast<float>(timestamp_us - last_timestamp_us_) * 1e-6F;
        last_timestamp_us_ = timestamp_us;
        if (dt > config_.max_dt) {
            dt = config_.max_dt;
        }

        Vector3 rate = gyro - bias_;
        const float magnitude = specific_force.norm();
        if (std::abs(magnitude - config_.gravity) <= config_.accel_rejection * config_.gravity) {
            const Vector3 error = (specific_force * (1.0F / magnitude)).cross(AccelerometerModel::predict_direction(q_));
            bias_ -= error * (config_.ki * dt);
            rate += error * config_.kp;
        } else {
            ++accel_rejected_;
        }
        q_ = QuaternionKinematics::predict(q_, rate, dt);
        angular_velocity_ = gyro - bias_;
    }

    // Changes the gains and the rejection threshold from the next sample on;
    // the attitude and the bias estimate are kept.
    void set_gains(float kp, float ki, float accel_rejection)
    {
        config_.kp = kp;
        config_.ki = ki;
        config_.accel_rejection = accel_rejection;
    }

    [[nodiscard]] const Config& config() const { return config_; }
    [[nodiscard]] const Quaternion& attitude() const { return q_; }
    [[nodiscard]] const Vector3& gyro_bias() const { return bias_; }
    // The last body rate with the bias estimate removed [rad/s].
    [[nodiscard]] const Vector3& angular_velocity() const { return angular_velocity_; }
    // Samples whose accelerometer correction was skipped.
    [[nodiscard]] std::uint32_t accel_rejected() const { return accel_rejected_; }

private:
    Config config_;
    Quaternion q_;
    Vector3 bias_;
    Vector3 angular_velocity_;
    std::uint64_t last_timestamp_us_ = 0;
    bool have_timestamp_ = false;
    std::uint32_t accel_rejected_ = 0;
};

}  // namespace vk::estimation
