// SPDX-License-Identifier: GPL-3.0-or-later
// Error-state (multiplicative) Extended Kalman Filter for attitude and gyro
// bias (ADR-0014, spec 0012).
//
// The nominal state is the attitude quaternion (body to the north-east-down
// local level frame) and the 3-axis gyro bias in the body frame. The 6-state
// error vector is the NED-frame attitude error rotation vector (indices 0..2,
// about north, east and down, so q = exp(delta_theta / 2) * q_hat) and the
// body-frame gyro bias error (indices 3..5), with a 6x6 error covariance P
// kept symmetric and positive definite.
//
// Prediction integrates the bias-corrected gyro rate through quaternion
// kinematics and propagates P in continuous time with gyro rate noise and
// gyro bias random walk, scaled by the sample's time step.
//
// A generic measurement update takes an M-dimensional innovation, its Mx6
// Jacobian on the error state, its scalar variance per component and a gate
// threshold in standard deviations. When the Mahalanobis distance passes the
// gate, the error state is corrected, injected into the nominal quaternion
// and gyro bias, and reset to zero while P is updated in Joseph form.
#pragma once

#include <vk/estimation/math/matrix.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/estimation/models/magnetometer_model.hpp>
#include <vk/estimation/models/quaternion_kinematics.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace vk::estimation {

struct AttitudeEskfConfig {
    float gyro_noise = 1.1e-3F;               // [rad/s/sqrt(Hz)] continuous-time gyro rate noise density
    float gyro_bias_noise = 7.0e-5F;          // [rad/s^2/sqrt(Hz)] continuous-time gyro bias random walk density
    float accel_noise = 2.0F;                 // [m/s^2] accelerometer gravity noise
    float accel_rejection = 0.1F;             // skip gravity when | |a| - g | exceeds this fraction of g
    float accel_gate = 5.0F;                  // [SD] innovation gate for gravity
    float mag_noise = 0.1F;                   // [rad] magnetometer heading noise
    float mag_gate = 3.0F;                    // [SD] innovation gate for heading
    float mag_declination = 0.0F;             // [rad] magnetic declination
    float mag_rejection = 0.2F;               // skip mag when | |m| - ref | exceeds this fraction of ref
    float zero_rate_gate = 5.0F;              // [SD] innovation gate for zero rate update
    float initial_attitude_variance = 0.5F;   // [rad^2] initial attitude error variance per axis
    float initial_bias_variance = 1e-3F;      // [rad^2/s^2] initial gyro bias error variance per axis
    float gravity = standard_gravity;         // [m/s^2]
    float max_dt = 0.1F;                      // [s] longer gaps between samples are cut to this
};

class AttitudeEskf {
public:
    using Config = AttitudeEskfConfig;

    explicit AttitudeEskf(const Config& config = {}) : config_(config) { reset(); }

    // Starts from an attitude, gyro bias and explicit 6x6 error covariance; the next timestamped sample sets the time.
    void reset(const Quaternion& attitude, const Vector3& gyro_bias, const Matrix<6, 6>& covariance)
    {
        q_ = attitude.normalized();
        bias_ = gyro_bias;
        last_gyro_ = gyro_bias;
        angular_velocity_ = {};
        p_ = covariance.symmetric();
        have_timestamp_ = false;
    }

    // Starts from an attitude, gyro bias and diagonal attitude/bias error variances.
    void reset(const Quaternion& attitude, const Vector3& gyro_bias, float attitude_variance, float bias_variance)
    {
        reset(attitude, gyro_bias,
              Matrix<6, 6>::diagonal({attitude_variance, attitude_variance, attitude_variance, bias_variance,
                                      bias_variance, bias_variance}));
    }

    // Starts from an attitude and gyro bias with the configured initial variances.
    void reset(const Quaternion& attitude = Quaternion::identity(), const Vector3& gyro_bias = {})
    {
        reset(attitude, gyro_bias, config_.initial_attitude_variance, config_.initial_bias_variance);
    }

    // Starts levelled by a specific force measured at rest, with the given gyro bias and yaw [rad].
    // If yaw_variance is provided, sets P(2,2); otherwise leaves P(2,2) at initial_attitude_variance.
    void reset_from(const Vector3& specific_force, const Vector3& gyro_bias = {}, float yaw = 0.0F,
                    std::optional<float> yaw_variance = std::nullopt)
    {
        Euler e = AccelerometerModel::level_from(specific_force);
        e.yaw = yaw;
        reset(Quaternion::from_euler(e), gyro_bias);
        if (yaw_variance.has_value()) {
            p_(2, 2) = *yaw_variance;
        }
    }

    // Propagates the nominal state and the 6x6 error covariance over `dt` seconds [s] with body rate `gyro` [rad/s].
    void predict(const Vector3& gyro, float dt)
    {
        last_gyro_ = gyro;
        angular_velocity_ = gyro - bias_;
        if (!(dt > 0.0F) || !std::isfinite(dt)) {
            return;
        }
        if (dt > config_.max_dt) {
            dt = config_.max_dt;
        }

        Matrix<6, 6> f = Matrix<6, 6>::identity();
        f.set_block(0, 3, Matrix<3, 3>::rotation(q_) * (-dt));

        q_ = QuaternionKinematics::predict(q_, angular_velocity_, dt);

        p_ = (f * p_ * f.transposed()).symmetric();
        const float q_att = config_.gyro_noise * config_.gyro_noise * dt;
        const float q_bias = config_.gyro_bias_noise * config_.gyro_bias_noise * dt;
        for (std::size_t i = 0; i < 3; ++i) {
            p_(i, i) += q_att;
            p_(i + 3, i + 3) += q_bias;
        }
    }

    // Timestamped prediction [us]: the first sample, or one out of order, only sets the time.
    void predict(const Vector3& gyro, std::uint64_t timestamp_us)
    {
        if (!have_timestamp_ || timestamp_us <= last_timestamp_us_) {
            if (!have_timestamp_ || timestamp_us < last_timestamp_us_) {
                last_timestamp_us_ = timestamp_us;
            }
            have_timestamp_ = true;
            last_gyro_ = gyro;
            angular_velocity_ = gyro - bias_;
            return;
        }
        const float dt = static_cast<float>(timestamp_us - last_timestamp_us_) * 1e-6F;
        last_timestamp_us_ = timestamp_us;
        predict(gyro, dt);
    }

    // Generic measurement update: tests the innovation against `gate_sd` standard deviations,
    // and on acceptance updates the nominal state and covariance (Joseph form) and resets the error state.
    template <std::size_t M>
    bool update(const Matrix<M, 1>& innovation, const Matrix<M, 6>& jacobian, float variance, float gate_sd)
        requires(M == 1 || M == 2 || M == 3)
    {
        if (!(variance > 0.0F) || !std::isfinite(variance) || !(gate_sd > 0.0F) || !std::isfinite(gate_sd)) {
            return false;
        }
        for (std::size_t i = 0; i < M; ++i) {
            if (!std::isfinite(innovation(i, 0))) {
                return false;
            }
        }

        const Matrix<6, M> ph_t = p_ * jacobian.transposed();
        const Matrix<M, M> r = Matrix<M, M>::diagonal(variance);
        const Matrix<M, M> s = (jacobian * ph_t + r).symmetric();
        const std::optional<Matrix<M, M>> s_inv = s.inverted();
        if (!s_inv.has_value()) {
            return false;
        }
        const Matrix<M, M> inv = s_inv.value_or(Matrix<M, M>{});

        const Matrix<M, 1> s_inv_y = inv * innovation;
        const float nis = (innovation.transposed() * s_inv_y)(0, 0);
        if (!std::isfinite(nis) || nis < 0.0F || nis > gate_sd * gate_sd) {
            return false;
        }

        const Matrix<6, M> k = ph_t * inv;
        const Matrix<6, 1> dx = k * innovation;
        const Vector3 delta_theta{dx(0, 0), dx(1, 0), dx(2, 0)};
        const Vector3 delta_bias{dx(3, 0), dx(4, 0), dx(5, 0)};

        q_ = (Quaternion::from_rotation_vector(delta_theta) * q_).normalized();
        bias_ += delta_bias;
        angular_velocity_ = last_gyro_ - bias_;

        const Matrix<6, 6> i_kh = Matrix<6, 6>::identity() - k * jacobian;
        p_ = (i_kh * p_ * i_kh.transposed() + k * r * k.transposed()).symmetric();
        return true;
    }

    // Scalar (1D) measurement update.
    bool update(float innovation, const Matrix<1, 6>& jacobian, float variance, float gate_sd)
    {
        return update(Matrix<1, 1>::from_rows({{{{innovation}}}}), jacobian, variance, gate_sd);
    }

    // 3D vector measurement update.
    bool update(const Vector3& innovation, const Matrix<3, 6>& jacobian, float variance, float gate_sd)
    {
        return update(Matrix<3, 1>::from_vector(innovation), jacobian, variance, gate_sd);
    }

    // Measurement-struct overload for AccelerometerModel::Measurement and MagnetometerModel::Measurement.
    template <typename Measurement>
    bool update(const Measurement& measurement, float gate_sd)
    {
        return update(measurement.innovation, measurement.jacobian, measurement.variance, gate_sd);
    }

    // Fuses an accelerometer sample as a gravity measurement when within accel_rejection and accel_gate.
    bool fuse_gravity(const Vector3& specific_force)
    {
        const std::optional<GravityMeasurement> m = AccelerometerModel::measure_gravity(
            q_, specific_force, config_.accel_noise, config_.accel_rejection, config_.gravity);
        if (!m.has_value() || !update(m.value_or(GravityMeasurement{}), config_.accel_gate)) {
            ++accel_rejected_;
            return false;
        }
        ++accel_fused_;
        return true;
    }

    // Fuses a magnetometer sample as a heading measurement when within mag_rejection (if reference_strength > 0)
    // and mag_gate.
    bool fuse_heading(const Vector3& field, float reference_strength = 0.0F)
    {
        const std::optional<HeadingMeasurement> m = MagnetometerModel::measure_heading(
            q_, field, config_.mag_noise, config_.mag_declination, reference_strength, config_.mag_rejection);
        if (!m.has_value() || !update(m.value_or(HeadingMeasurement{}), config_.mag_gate)) {
            ++mag_rejected_;
            return false;
        }
        ++mag_fused_;
        return true;
    }

    // Fuses an averaged gyro reading while at rest as a direct measurement of the gyro bias.
    bool fuse_zero_rate(const Vector3& mean_gyro, float variance)
    {
        Matrix<3, 6> jacobian{};
        jacobian.set_block(0, 3, Matrix<3, 3>::identity());
        const Vector3 innovation = mean_gyro - bias_;
        if (!update(innovation, jacobian, variance, config_.zero_rate_gate)) {
            ++zero_rate_rejected_;
            return false;
        }
        ++zero_rate_fused_;
        return true;
    }

    void set_config(const Config& config) { config_ = config; }

    [[nodiscard]] const Config& config() const { return config_; }
    [[nodiscard]] const Quaternion& attitude() const { return q_; }
    [[nodiscard]] const Vector3& gyro_bias() const { return bias_; }
    // The last body rate with the bias estimate removed [rad/s].
    [[nodiscard]] const Vector3& angular_velocity() const { return angular_velocity_; }
    [[nodiscard]] const Matrix<6, 6>& covariance() const { return p_; }
    // NED-frame attitude error variances [rad^2] about north, east and down (P(0,0), P(1,1), P(2,2));
    // P(2,2) is the yaw variance.
    [[nodiscard]] Vector3 attitude_variance() const { return {p_(0, 0), p_(1, 1), p_(2, 2)}; }
    // Gyro bias error variances [rad^2/s^2] from the diagonal of P.
    [[nodiscard]] Vector3 bias_variance() const { return {p_(3, 3), p_(4, 4), p_(5, 5)}; }
    [[nodiscard]] std::uint32_t accel_rejected() const { return accel_rejected_; }
    [[nodiscard]] std::uint32_t accel_fused() const { return accel_fused_; }
    [[nodiscard]] std::uint32_t mag_rejected() const { return mag_rejected_; }
    [[nodiscard]] std::uint32_t mag_fused() const { return mag_fused_; }
    [[nodiscard]] std::uint32_t zero_rate_rejected() const { return zero_rate_rejected_; }
    [[nodiscard]] std::uint32_t zero_rate_fused() const { return zero_rate_fused_; }

private:
    Config config_;
    Quaternion q_;
    Vector3 bias_;
    Vector3 last_gyro_;
    Vector3 angular_velocity_;
    Matrix<6, 6> p_;
    std::uint64_t last_timestamp_us_ = 0;
    bool have_timestamp_ = false;
    std::uint32_t accel_rejected_ = 0;
    std::uint32_t accel_fused_ = 0;
    std::uint32_t mag_rejected_ = 0;
    std::uint32_t mag_fused_ = 0;
    std::uint32_t zero_rate_rejected_ = 0;
    std::uint32_t zero_rate_fused_ = 0;
};

}  // namespace vk::estimation
