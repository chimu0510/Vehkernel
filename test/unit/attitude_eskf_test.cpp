// SPDX-License-Identifier: GPL-3.0-or-later
// The error-state EKF attitude filter and its gravity and heading measurement
// models (ADR-0014, spec 0012).
#include <vk/estimation/filters/attitude_eskf.hpp>
#include <vk/estimation/math/matrix.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/estimation/models/magnetometer_model.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <random>

namespace {

using vk::estimation::AccelerometerModel;
using vk::estimation::AttitudeEskf;
using vk::estimation::Euler;
using vk::estimation::MagnetometerModel;
using vk::estimation::Matrix;
using vk::estimation::Matrix6;
using vk::estimation::Quaternion;
using vk::estimation::Vector3;

constexpr float pi = std::numbers::pi_v<float>;
constexpr float dt_s = 0.005F;  // 200 Hz
const Vector3 earth_field_ned{30.0F, 0.0F, 30.0F};

float deg(float degrees)
{
    return degrees * pi / 180.0F;
}

float to_deg(float radians)
{
    return radians * 180.0F / pi;
}

bool is_symmetric(const Matrix6& p, float tolerance = 0.0F)
{
    for (std::size_t r = 0; r < 6; ++r) {
        for (std::size_t c = r + 1; c < 6; ++c) {
            if (!std::isfinite(p(r, c)) || std::abs(p(r, c) - p(c, r)) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

// Cholesky factorization P = L * L^T: succeeds iff the symmetric matrix P is strictly positive definite.
bool is_positive_definite(const Matrix6& p)
{
    Matrix6 l{};
    for (std::size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(p(i, i)) || !(p(i, i) > 0.0F)) {
            return false;
        }
        for (std::size_t j = 0; j <= i; ++j) {
            float sum = p(i, j);
            for (std::size_t k = 0; k < j; ++k) {
                sum -= l(i, k) * l(j, k);
            }
            if (i == j) {
                if (!(sum > 0.0F) || !std::isfinite(sum)) {
                    return false;
                }
                l(i, j) = std::sqrt(sum);
            } else {
                l(i, j) = sum / l(j, j);
            }
        }
    }
    return true;
}

TEST(AttitudeEskf, CovarianceGrowsWithNoMeasurements)
{
    AttitudeEskf::Config config{};
    config.gyro_noise = 0.015F;
    config.gyro_bias_noise = 0.001F;
    AttitudeEskf eskf(config);
    eskf.reset(Quaternion::identity(), {}, Matrix6::zero());

    constexpr int steps = 400;  // 2.0 s at 200 Hz
    for (int i = 0; i < steps; ++i) {
        eskf.predict({}, dt_s);
    }

    const float total_time = static_cast<float>(steps) * dt_s;
    const float q_att = config.gyro_noise * config.gyro_noise;
    const float q_bias = config.gyro_bias_noise * config.gyro_bias_noise;
    const auto n = static_cast<float>(steps);
    // Discrete sum: sum_{k=0}^{N-1} k^2 = (N - 1) * N * (2N - 1) / 6 (~ T^3 / 3).
    const float expected_bias_var = q_bias * total_time;
    const float expected_att_var =
        q_att * total_time + q_bias * (dt_s * dt_s * dt_s) * ((n - 1.0F) * n * (2.0F * n - 1.0F) / 6.0F);
    const float expected_cross = -q_bias * (dt_s * dt_s) * ((n - 1.0F) * n / 2.0F);

    const Matrix6& p = eskf.covariance();
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(p(i, i), expected_att_var, 1e-8F);
        EXPECT_NEAR(p(i + 3, i + 3), expected_bias_var, 1e-10F);
        EXPECT_NEAR(p(i, i + 3), expected_cross, 1e-9F);
        EXPECT_NEAR(p(i + 3, i), expected_cross, 1e-9F);
    }
    EXPECT_TRUE(is_symmetric(p));
    EXPECT_TRUE(is_positive_definite(p));
}

TEST(AttitudeEskf, GravityConvergesRollAndPitchFrom30DegreesAndEstimatesXYBias)
{
    const Quaternion truth = Quaternion::from_euler({.roll = deg(30), .pitch = deg(-30), .yaw = 0.0F});
    const Vector3 true_bias{0.02F, -0.015F, 0.0F};
    const Vector3 accel = AccelerometerModel::predict(truth);

    AttitudeEskf eskf;
    eskf.reset(Quaternion::identity(), {});

    // 40 s at 200 Hz: both the 30 deg roll/pitch error and the x/y gyro bias settle.
    for (int i = 0; i < 8000; ++i) {
        eskf.predict(true_bias, dt_s);
        ASSERT_TRUE(eskf.fuse_gravity(accel));
    }

    const Euler e = eskf.attitude().euler();
    EXPECT_NEAR(to_deg(e.roll), 30.0F, 0.5F);
    EXPECT_NEAR(to_deg(e.pitch), -30.0F, 0.5F);
    EXPECT_NEAR(eskf.gyro_bias().x, true_bias.x, 0.002F);
    EXPECT_NEAR(eskf.gyro_bias().y, true_bias.y, 0.002F);
    EXPECT_NEAR(eskf.angular_velocity().x, 0.0F, 0.002F);
    EXPECT_NEAR(eskf.angular_velocity().y, 0.0F, 0.002F);
    EXPECT_EQ(eskf.accel_rejected(), 0U);
    EXPECT_EQ(eskf.accel_fused(), 8000U);
}

TEST(AttitudeEskf, HeadingConvergesYawAndEstimatesZBias)
{
    const Quaternion truth = Quaternion::from_euler({.roll = deg(10), .pitch = deg(-10), .yaw = deg(45)});
    const Vector3 true_bias{0.0F, 0.0F, 0.02F};
    const Vector3 accel = AccelerometerModel::predict(truth);
    const Vector3 field = MagnetometerModel::predict(truth, earth_field_ned);

    AttitudeEskf eskf;
    eskf.reset(Quaternion::from_euler({.roll = deg(10), .pitch = deg(-10), .yaw = 0.0F}), {});

    // 40 s with IMU at 200 Hz and magnetometer at 100 Hz.
    for (int i = 0; i < 8000; ++i) {
        eskf.predict(true_bias, dt_s);
        ASSERT_TRUE(eskf.fuse_gravity(accel));
        if (i % 2 == 0) {
            ASSERT_TRUE(eskf.fuse_heading(field));
        }
    }

    const Euler e = eskf.attitude().euler();
    EXPECT_NEAR(to_deg(e.roll), 10.0F, 0.5F);
    EXPECT_NEAR(to_deg(e.pitch), -10.0F, 0.5F);
    EXPECT_NEAR(to_deg(e.yaw), 45.0F, 0.5F);
    EXPECT_NEAR(eskf.gyro_bias().z, true_bias.z, 0.002F);
    EXPECT_NEAR(eskf.angular_velocity().z, 0.0F, 0.002F);
    EXPECT_EQ(eskf.mag_rejected(), 0U);
    EXPECT_EQ(eskf.mag_fused(), 4000U);
}

TEST(AttitudeEskf, HeadingJacobianLeavesRollAndPitchUntouched)
{
    AttitudeEskf eskf;
    eskf.reset(Quaternion::from_euler({.roll = deg(20), .pitch = deg(-15), .yaw = deg(10)}), {0.01F, -0.01F, 0.0F});
    const Euler before_euler = eskf.attitude().euler();
    const Vector3 before_bias = eskf.gyro_bias();
    const Matrix6 before_p = eskf.covariance();

    const Quaternion target_yaw = Quaternion::from_euler({.roll = deg(20), .pitch = deg(-15), .yaw = deg(35)});
    const auto m =
        MagnetometerModel::heading_measurement(eskf.attitude(), MagnetometerModel::predict(target_yaw, earth_field_ned),
                                               eskf.config().mag_noise);
    EXPECT_FLOAT_EQ(m.jacobian(0, 0), 0.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 1), 0.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 2), 1.0F);

    ASSERT_TRUE(eskf.update(m, eskf.config().mag_gate));

    const Euler after_euler = eskf.attitude().euler();
    EXPECT_NEAR(after_euler.roll, before_euler.roll, 1e-5F);
    EXPECT_NEAR(after_euler.pitch, before_euler.pitch, 1e-5F);
    EXPECT_GT(after_euler.yaw, before_euler.yaw);
    EXPECT_FLOAT_EQ(eskf.gyro_bias().x, before_bias.x);
    EXPECT_FLOAT_EQ(eskf.gyro_bias().y, before_bias.y);
    EXPECT_FLOAT_EQ(eskf.covariance()(0, 0), before_p(0, 0));
    EXPECT_FLOAT_EQ(eskf.covariance()(1, 1), before_p(1, 1));
    EXPECT_LT(eskf.covariance()(2, 2), before_p(2, 2));
}

TEST(AttitudeEskf, HeadingWrapsAcrossPlusMinusPi)
{
    // Estimate at +170 deg, truth at -170 deg: shortest turn is +20 deg across +pi.
    {
        AttitudeEskf eskf;
        eskf.reset(Quaternion::from_euler({.yaw = deg(170)}), {});
        const Quaternion truth = Quaternion::from_euler({.yaw = deg(-170)});
        const Vector3 field = MagnetometerModel::predict(truth, earth_field_ned);
        const auto m = MagnetometerModel::heading_measurement(eskf.attitude(), field, eskf.config().mag_noise);
        EXPECT_NEAR(to_deg(m.innovation), 20.0F, 1e-2F);

        for (int i = 0; i < 50; ++i) {
            ASSERT_TRUE(eskf.fuse_heading(field));
        }
        EXPECT_NEAR(to_deg(MagnetometerModel::wrap_pi(eskf.attitude().euler().yaw - deg(-170))), 0.0F, 0.2F);
    }

    // Estimate at -170 deg, truth at +170 deg: shortest turn is -20 deg across -pi.
    {
        AttitudeEskf eskf;
        eskf.reset(Quaternion::from_euler({.yaw = deg(-170)}), {});
        const Quaternion truth = Quaternion::from_euler({.yaw = deg(170)});
        const Vector3 field = MagnetometerModel::predict(truth, earth_field_ned);
        const auto m = MagnetometerModel::heading_measurement(eskf.attitude(), field, eskf.config().mag_noise);
        EXPECT_NEAR(to_deg(m.innovation), -20.0F, 1e-2F);

        for (int i = 0; i < 50; ++i) {
            ASSERT_TRUE(eskf.fuse_heading(field));
        }
        EXPECT_NEAR(to_deg(MagnetometerModel::wrap_pi(eskf.attitude().euler().yaw - deg(170))), 0.0F, 0.2F);
    }
}

TEST(AttitudeEskf, GatedOutMeasurementLeavesStateAndCovarianceUnchanged)
{
    AttitudeEskf eskf;
    const Quaternion initial_q = Quaternion::from_euler({.roll = deg(5), .pitch = deg(-8), .yaw = deg(25)});
    const Vector3 initial_bias{0.005F, -0.004F, 0.003F};
    // Small attitude variance (1 deg)^2 so a 90 deg heading jump is far outside the 3 SD gate.
    eskf.reset(initial_q, initial_bias, 3e-4F, 1e-5F);
    eskf.predict(initial_bias, dt_s);

    const Quaternion q_before = eskf.attitude();
    const Vector3 bias_before = eskf.gyro_bias();
    const Matrix6 p_before = eskf.covariance();

    // 1. Heading outlier gated out.
    const Quaternion outlier_yaw = Quaternion::from_euler({.roll = deg(5), .pitch = deg(-8), .yaw = deg(120)});
    EXPECT_FALSE(eskf.fuse_heading(MagnetometerModel::predict(outlier_yaw, earth_field_ned)));
    EXPECT_EQ(eskf.mag_rejected(), 1U);
    EXPECT_EQ(eskf.mag_fused(), 0U);

    // 2. Gravity outlier (90 deg tilt with 1g magnitude) gated out.
    const Quaternion outlier_tilt = Quaternion::from_euler({.roll = deg(90), .pitch = deg(-8), .yaw = deg(25)});
    EXPECT_FALSE(eskf.fuse_gravity(AccelerometerModel::predict(outlier_tilt)));
    EXPECT_EQ(eskf.accel_rejected(), 1U);
    EXPECT_EQ(eskf.accel_fused(), 0U);

    // 3. Direct generic update outside the gate.
    const auto h = Matrix<1, 6>::from_rows({{{{0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F}}}});
    EXPECT_FALSE(eskf.update(1.5F, h, 0.01F, 3.0F));

    EXPECT_FLOAT_EQ(eskf.attitude().w, q_before.w);
    EXPECT_FLOAT_EQ(eskf.attitude().x, q_before.x);
    EXPECT_FLOAT_EQ(eskf.attitude().y, q_before.y);
    EXPECT_FLOAT_EQ(eskf.attitude().z, q_before.z);
    EXPECT_FLOAT_EQ(eskf.gyro_bias().x, bias_before.x);
    EXPECT_FLOAT_EQ(eskf.gyro_bias().y, bias_before.y);
    EXPECT_FLOAT_EQ(eskf.gyro_bias().z, bias_before.z);
    for (std::size_t r = 0; r < 6; ++r) {
        for (std::size_t c = 0; c < 6; ++c) {
            EXPECT_FLOAT_EQ(eskf.covariance()(r, c), p_before(r, c));
        }
    }
}

TEST(AttitudeEskf, CovarianceStaysSymmetricAndPositiveOverLongRandomRun)
{
    AttitudeEskf eskf;
    Quaternion truth = Quaternion::identity();
    const Vector3 true_bias{0.01F, -0.01F, 0.005F};

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> rate_dist(-1.5F, 1.5F);
    std::uniform_real_distribution<float> dt_dist(0.001F, 0.015F);
    std::normal_distribution<float> gyro_noise(0.0F, 0.015F);
    std::normal_distribution<float> accel_noise(0.0F, 0.3F);
    std::normal_distribution<float> mag_noise(0.0F, 1.5F);

    for (int i = 0; i < 5000; ++i) {
        const float step_dt = dt_dist(rng);
        const Vector3 true_rate{rate_dist(rng), rate_dist(rng), rate_dist(rng)};
        truth = truth.integrated(true_rate, step_dt);

        const Vector3 measured_gyro =
            true_rate + true_bias + Vector3{gyro_noise(rng), gyro_noise(rng), gyro_noise(rng)};
        eskf.predict(measured_gyro, step_dt);
        ASSERT_TRUE(is_symmetric(eskf.covariance()));
        ASSERT_TRUE(is_positive_definite(eskf.covariance()));

        const Vector3 measured_accel =
            AccelerometerModel::predict(truth) + Vector3{accel_noise(rng), accel_noise(rng), accel_noise(rng)};
        eskf.fuse_gravity(measured_accel);
        ASSERT_TRUE(is_symmetric(eskf.covariance()));
        ASSERT_TRUE(is_positive_definite(eskf.covariance()));

        if (i % 2 == 0) {
            const Vector3 measured_mag =
                MagnetometerModel::predict(truth, earth_field_ned) + Vector3{mag_noise(rng), mag_noise(rng), mag_noise(rng)};
            eskf.fuse_heading(measured_mag, earth_field_ned.norm());
            ASSERT_TRUE(is_symmetric(eskf.covariance()));
            ASSERT_TRUE(is_positive_definite(eskf.covariance()));
        }
    }

    EXPECT_NEAR(eskf.attitude().norm(), 1.0F, 1e-5F);
    EXPECT_GT(eskf.accel_fused(), 4000U);
    EXPECT_GT(eskf.mag_fused(), 2000U);
}

}  // namespace
