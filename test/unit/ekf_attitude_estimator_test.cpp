// SPDX-License-Identifier: GPL-3.0-or-later
// Unit tests for the EKF attitude estimator (ADR-0014, spec 0012):
// alignment with and without heading fusion, primary selection, AUTO mag type,
// field strength rejection, missing magnetometer warning, and restart.
#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/estimation/attitude_estimator.hpp>
#include <vk/estimation/ekf_attitude_estimator.hpp>
#include <vk/estimation/estimator_selection.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/estimator_attitude.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/magnetometer_sample.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/params/params.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numbers>
#include <string_view>

namespace {

using vk::estimation::AccelerometerModel;
using vk::estimation::AttitudeEstimator;
using vk::estimation::EkfAttitudeEstimator;
using vk::estimation::EkfMagType;
using vk::estimation::PrimaryEstimator;
using vk::estimation::Quaternion;
using vk::estimation::Vector3;
using vk::estimation::is_primary_fallback;
using vk::estimation::primary_estimator;
using vk::estimation::resolve_mag_type;
using vk::estimation::resolve_primary;

constexpr float pi = std::numbers::pi_v<float>;
constexpr std::uint8_t imu_instance = 0;
constexpr std::uint8_t mag_instance = 0;
constexpr std::uint64_t period_us = 5'000;  // 200 Hz IMU period

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

const Vector3 level_accel = AccelerometerModel::predict(Quaternion::identity());

class EkfAttitudeEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        // Drop what earlier tests left on attitude topics.
        vk::msg::VehicleAttitude stale_primary{};
        drain_primary(stale_primary);
        vk::msg::EstimatorAttitude stale_sec{};
        drain_secondary(0, stale_sec);
        drain_secondary(1, stale_sec);
    }

    void TearDown() override
    {
        auto& parameters = vk::params::parameters();
        for (std::size_t i = 0; i < parameters.count(); ++i) {
            parameters.set(i, parameters.info(i).default_value);
        }
    }

    static void set_float(std::string_view name, float value)
    {
        ASSERT_EQ(vk::params::parameters().set(name, vk::core::ParamValue::from_float(value)),
                  vk::core::ParamResult::Ok);
    }

    static void set_int(std::string_view name, std::int32_t value)
    {
        ASSERT_EQ(vk::params::parameters().set(name, vk::core::ParamValue::from_int(value)),
                  vk::core::ParamResult::Ok);
    }

    // Publishes IMU samples 5 ms apart.
    void feed_imu(int count, const Vector3& accel, const Vector3& gyro, int per_run = 1)
    {
        for (int i = 0; i < count; ++i) {
            vk::msg::ImuSample sample{};
            sample.timestamp = fake_now;
            sample.device_instance = imu_instance;
            sample.accel = accel.array();
            sample.gyro = gyro.array();
            imu_pub.publish(sample);
            fake_now += period_us;
            if ((i + 1) % per_run == 0) {
                queue.process();
            }
        }
        queue.process();
    }

    // Publishes a magnetometer sample stamped at `fake_now`.
    void send_mag(const Vector3& field)
    {
        vk::msg::MagnetometerSample sample{};
        sample.timestamp = fake_now;
        sample.device_instance = mag_instance;
        sample.field = field.array();
        mag_pub.publish(sample);
    }

    // Publishes a body velocity measurement stamped at `fake_now`.
    void send_velocity(float vx = 0.0F, float vy = 0.0F, float yaw_rate = 0.0F)
    {
        vk::msg::BodyVelocityMeasurement sample{};
        sample.timestamp = fake_now;
        sample.timestamp_sample = fake_now;
        sample.vx = vx;
        sample.vy = vy;
        sample.yaw_rate = yaw_rate;
        velocity_pub.publish(sample);
    }

    // Publishes IMU samples while sending a body velocity measurement alongside each IMU sample.
    void feed_imu_with_velocity(int count, const Vector3& accel, const Vector3& gyro,
                                float vx = 0.0F, float vy = 0.0F, float yaw_rate = 0.0F)
    {
        for (int i = 0; i < count; ++i) {
            send_velocity(vx, vy, yaw_rate);
            vk::msg::ImuSample sample{};
            sample.timestamp = fake_now;
            sample.device_instance = imu_instance;
            sample.accel = accel.array();
            sample.gyro = gyro.array();
            imu_pub.publish(sample);
            fake_now += period_us;
            queue.process();
        }
        queue.process();
    }

    // Publishes IMU samples while sending a magnetometer sample alongside every other IMU sample (100 Hz).
    void feed_both(int count, const Vector3& accel, const Vector3& gyro, const Vector3& mag_field)
    {
        for (int i = 0; i < count; ++i) {
            if (i % 2 == 0) {
                send_mag(mag_field);
            }
            vk::msg::ImuSample sample{};
            sample.timestamp = fake_now;
            sample.device_instance = imu_instance;
            sample.accel = accel.array();
            sample.gyro = gyro.array();
            imu_pub.publish(sample);
            fake_now += period_us;
            queue.process();
        }
        queue.process();
    }

    int drain_primary(vk::msg::VehicleAttitude& last)
    {
        const std::uint64_t gen = vk::core::topic<vk::msg::VehicleAttitude>(0).generation();
        const auto count = static_cast<int>(gen - seen_primary_);
        seen_primary_ = gen;
        while (primary_sub.copy(last)) {
        }
        return count;
    }

    int drain_secondary(std::size_t instance, vk::msg::EstimatorAttitude& last)
    {
        const std::uint64_t gen = vk::core::topic<vk::msg::EstimatorAttitude>(instance).generation();
        std::uint64_t& seen = (instance == 0) ? seen_sec_0_ : seen_sec_1_;
        const auto count = static_cast<int>(gen - seen);
        seen = gen;
        auto& sub = (instance == 0) ? sec_sub_0 : sec_sub_1;
        while (sub.copy(last)) {
        }
        return count;
    }

    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    vk::core::Publication<vk::msg::ImuSample> imu_pub{imu_instance};
    vk::core::Publication<vk::msg::MagnetometerSample> mag_pub{mag_instance};
    vk::core::Publication<vk::msg::BodyVelocityMeasurement> velocity_pub{0};
    vk::core::Subscription<vk::msg::VehicleAttitude> primary_sub{0};
    vk::core::Subscription<vk::msg::EstimatorAttitude> sec_sub_0{0};
    vk::core::Subscription<vk::msg::EstimatorAttitude> sec_sub_1{1};
    std::uint64_t seen_primary_ = 0;
    std::uint64_t seen_sec_0_ = 0;
    std::uint64_t seen_sec_1_ = 0;
};

// 1. Alignment without heading fusion: roll and pitch from mean specific force, yaw zero,
//    bias from mean rate, nothing published while aligning.
TEST_F(EkfAttitudeEstimatorTest, AlignmentWithoutHeadingFusion)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    const Quaternion tilted = Quaternion::from_euler({.roll = 0.25F, .pitch = -0.15F, .yaw = 1.0F});
    const Vector3 tilted_accel = AccelerometerModel::predict(tilted);
    const Vector3 bias{0.012F, -0.018F, 0.006F};

    // 100 samples = 0.5 s: half the alignment time. Nothing published yet.
    feed_imu(100, tilted_accel, bias);
    vk::msg::VehicleAttitude out{};
    EXPECT_EQ(drain_primary(out), 0);
    EXPECT_EQ(estimator.state(), EkfAttitudeEstimator::State::Aligning);

    // Another 101 samples (total 201 samples > 1.0 s) completes alignment.
    feed_imu(101, tilted_accel, bias);
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    // Feed 10 samples to verify published estimate.
    feed_imu(10, tilted_accel, bias);
    EXPECT_EQ(drain_primary(out), 10);

    const auto e = Quaternion::from(out.q).euler();
    // 1e-3F absorbs floating-point trigonometric discretization and specific force normalization.
    EXPECT_NEAR(e.roll, 0.25F, 1e-3F);
    EXPECT_NEAR(e.pitch, -0.15F, 1e-3F);
    // 1e-3F absorbs floating-point roundoff in quaternion attitude initialization with yaw at zero.
    EXPECT_NEAR(e.yaw, 0.0F, 1e-3F);
    // 1e-3F absorbs arithmetic division rounding over accumulated sample sums.
    EXPECT_NEAR(out.gyro_bias[0], bias.x, 1e-3F);
    EXPECT_NEAR(out.gyro_bias[1], bias.y, 1e-3F);
    EXPECT_NEAR(out.gyro_bias[2], bias.z, 1e-3F);
    // 1e-3F absorbs residual rate calculation error after gyro bias subtraction.
    EXPECT_NEAR(out.angular_velocity[0], 0.0F, 1e-3F);
    EXPECT_NEAR(out.angular_velocity[1], 0.0F, 1e-3F);
    EXPECT_NEAR(out.angular_velocity[2], 0.0F, 1e-3F);

    EXPECT_EQ(out.estimator_id, 2U);
    EXPECT_FALSE(out.mag_missing);
    EXPECT_EQ(out.mag_rejected, 0U);
    EXPECT_EQ(out.mag_fused, 0U);
    EXPECT_GT(out.variance[0], 0.0F);
    EXPECT_GT(out.variance[1], 0.0F);
    EXPECT_GT(out.variance[2], 0.0F);
}

// 2. Alignment with heading fusion: yaw is mean magnetometer heading (field pointing east + declination),
//    field strength reference is mean strength.
TEST_F(EkfAttitudeEstimatorTest, AlignmentWithHeadingFusion)
{
    // Magnetic declination: 10 degrees. EKF_MAG_DECL is positive east, and yaw relative to true north
    // is the magnetic heading plus the declination.
    constexpr float decl_deg = 10.0F;
    constexpr float decl_rad = decl_deg * (pi / 180.0F);
    set_float("EKF_MAG_DECL", decl_deg);

    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Heading,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // Body at level. A field pointing East in NED body frame: (0, 30, 0) uT.
    // Levelled magnetic heading is atan2(-y, x) = atan2(-30, 0) = -pi/2 (-90 deg).
    // Initial yaw = wrap_pi(-pi/2 + decl_rad).
    const Vector3 east_field{0.0F, 30.0F, 0.0F};
    const float expected_yaw = -pi / 2.0F + decl_rad;

    feed_both(201, level_accel, {}, east_field);
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    // 1e-3F absorbs floating-point norm computation rounding for the 30.0 uT vector.
    EXPECT_NEAR(estimator.reference_field_strength(), 30.0F, 1e-3F);
    EXPECT_FALSE(estimator.mag_missing());

    vk::msg::VehicleAttitude out{};
    feed_both(10, level_accel, {}, east_field);
    EXPECT_EQ(drain_primary(out), 10);

    const auto e = Quaternion::from(out.q).euler();
    // 1e-3F absorbs atan2 and levelling trigonometric precision.
    EXPECT_NEAR(e.yaw, expected_yaw, 1e-3F);
    EXPECT_FALSE(out.mag_missing);
}

// 3. Primary selection pure function for every EST_PRIMARY value and built-in estimator combination.
TEST_F(EkfAttitudeEstimatorTest, PrimarySelectionPureFunction)
{
    // Case 1: Both Mahony and EKF built in
    EXPECT_EQ(resolve_primary(1, true, true), PrimaryEstimator::Mahony);
    EXPECT_FALSE(is_primary_fallback(1, PrimaryEstimator::Mahony));

    EXPECT_EQ(resolve_primary(2, true, true), PrimaryEstimator::Ekf);
    EXPECT_FALSE(is_primary_fallback(2, PrimaryEstimator::Ekf));

    // Unknown value falls back to default EKF
    EXPECT_EQ(resolve_primary(99, true, true), PrimaryEstimator::Ekf);

    // Case 2: Only Mahony built in (EKF not built in)
    // EST_PRIMARY EKF (2) must fall back to Mahony!
    EXPECT_EQ(resolve_primary(2, true, false), PrimaryEstimator::Mahony);
    EXPECT_TRUE(is_primary_fallback(2, PrimaryEstimator::Mahony));
    EXPECT_EQ(resolve_primary(1, true, false), PrimaryEstimator::Mahony);
    EXPECT_FALSE(is_primary_fallback(1, PrimaryEstimator::Mahony));

    // Case 3: Only EKF built in (Mahony not built in)
    // EST_PRIMARY MAHONY (1) must fall back to EKF!
    EXPECT_EQ(resolve_primary(1, false, true), PrimaryEstimator::Ekf);
    EXPECT_TRUE(is_primary_fallback(1, PrimaryEstimator::Ekf));
    EXPECT_EQ(resolve_primary(2, false, true), PrimaryEstimator::Ekf);
    EXPECT_FALSE(is_primary_fallback(2, PrimaryEstimator::Ekf));

    // Non-constexpr primary_estimator() resolves using built-in flags and EST_PRIMARY
    EXPECT_EQ(primary_estimator(), PrimaryEstimator::Ekf);
}

// 4. Primary selection puts each estimator on the right topic and instance.
TEST_F(EkfAttitudeEstimatorTest, EachEstimatorPublishesOnTheRightTopicAndInstance)
{
    // EKF primary: publishes on vehicle_attitude; nothing on estimator_attitude instance 1.
    {
        EkfAttitudeEstimator::Config ekf_primary_cfg{
            .imu_instance = imu_instance,
            .primary = true,
            .mag_type = EkfMagType::Off,
        };
        EkfAttitudeEstimator ekf(ekf_primary_cfg, queue);
        ekf.start();
        feed_imu(201, level_accel, {});
        feed_imu(5, level_accel, {});

        vk::msg::VehicleAttitude prim{};
        EXPECT_EQ(drain_primary(prim), 5);
        EXPECT_EQ(prim.estimator_id, 2U);

        vk::msg::EstimatorAttitude sec{};
        EXPECT_EQ(drain_secondary(1, sec), 0);
    }

    // EKF secondary: publishes on estimator_attitude instance 1; nothing on vehicle_attitude.
    {
        EkfAttitudeEstimator::Config ekf_sec_cfg{
            .imu_instance = imu_instance,
            .primary = false,
            .mag_type = EkfMagType::Off,
        };
        EkfAttitudeEstimator ekf(ekf_sec_cfg, queue);
        ekf.start();
        feed_imu(201, level_accel, {});
        feed_imu(5, level_accel, {});

        vk::msg::VehicleAttitude prim{};
        EXPECT_EQ(drain_primary(prim), 0);

        vk::msg::EstimatorAttitude sec{};
        EXPECT_EQ(drain_secondary(1, sec), 5);
        EXPECT_EQ(sec.estimator_id, 2U);
    }

    // Mahony secondary: publishes on estimator_attitude instance 0; nothing on vehicle_attitude.
    {
        AttitudeEstimator::Config mahony_sec_cfg{
            .imu_instance = imu_instance,
            .primary = false,
        };
        AttitudeEstimator mahony(mahony_sec_cfg, queue);
        mahony.start();
        feed_imu(201, level_accel, {});
        feed_imu(5, level_accel, {});

        vk::msg::VehicleAttitude prim{};
        EXPECT_EQ(drain_primary(prim), 0);

        vk::msg::EstimatorAttitude sec{};
        EXPECT_EQ(drain_secondary(0, sec), 5);
        EXPECT_EQ(sec.estimator_id, 1U);
    }
}

// 5. AUTO resolves to off for GROUND.
TEST_F(EkfAttitudeEstimatorTest, AutoResolvesToOffForGround)
{
    using vk::msg::VehicleType;

    // AUTO resolves to Off for Ground
    EXPECT_EQ(resolve_mag_type(EkfMagType::Auto, VehicleType::Ground), EkfMagType::Off);

    // AUTO resolves to Heading for Surface and Underwater
    EXPECT_EQ(resolve_mag_type(EkfMagType::Auto, VehicleType::Surface), EkfMagType::Heading);
    EXPECT_EQ(resolve_mag_type(EkfMagType::Auto, VehicleType::Underwater), EkfMagType::Heading);

    // Explicit OFF remains Off regardless of vehicle type
    EXPECT_EQ(resolve_mag_type(EkfMagType::Off, VehicleType::Ground), EkfMagType::Off);
    EXPECT_EQ(resolve_mag_type(EkfMagType::Off, VehicleType::Surface), EkfMagType::Off);

    // Explicit HEADING remains Heading regardless of vehicle type
    EXPECT_EQ(resolve_mag_type(EkfMagType::Heading, VehicleType::Ground), EkfMagType::Heading);
    EXPECT_EQ(resolve_mag_type(EkfMagType::Heading, VehicleType::Surface), EkfMagType::Heading);

    // Integer overload
    EXPECT_EQ(resolve_mag_type(0, 0), EkfMagType::Off);
    EXPECT_EQ(resolve_mag_type(0, 1), EkfMagType::Heading);
}

// 6. Field strength rejection: samples differing by more than EKF_MAG_REJ are rejected and
//    do not change the yaw; samples within it are fused.
TEST_F(EkfAttitudeEstimatorTest, FieldStrengthRejection)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .filter = {.mag_noise = 0.05F, .mag_gate = 10.0F, .mag_rejection = 0.2F},
        .primary = true,
        .mag_type = EkfMagType::Heading,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // Align with 30 uT field pointing North: (30, 0, 0) -> heading 0.
    const Vector3 ref_field{30.0F, 0.0F, 0.0F};
    feed_both(201, level_accel, {}, ref_field);
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    vk::msg::VehicleAttitude out{};
    feed_imu(1, level_accel, {});
    drain_primary(out);
    const float yaw_before = Quaternion::from(out.q).euler().yaw;

    // Send a distorted field: norm 45 uT (differs from 30 uT by 50% > 20% EKF_MAG_REJ).
    // Pointing East: if fused, it would strongly pull yaw.
    const Vector3 bad_field{0.0F, 45.0F, 0.0F};
    send_mag(bad_field);
    feed_imu(1, level_accel, {});
    drain_primary(out);

    EXPECT_EQ(out.mag_rejected, 1U);
    EXPECT_EQ(out.mag_fused, 0U);
    // 1e-4F absorbs gyro integration numerical precision over a single 5 ms sample period.
    EXPECT_NEAR(Quaternion::from(out.q).euler().yaw, yaw_before, 1e-4F);

    // Send a valid field: norm 31 uT (differs from 30 uT by 3.3% <= 20% EKF_MAG_REJ).
    const Vector3 good_field{31.0F, 0.0F, 0.0F};
    send_mag(good_field);
    feed_imu(1, level_accel, {});
    drain_primary(out);

    EXPECT_EQ(out.mag_rejected, 1U);
    EXPECT_EQ(out.mag_fused, 1U);
}

// 7. Heading fusion on and no magnetometer sample while aligning:
//    yaw zero, mag_missing true, no magnetometer fused.
TEST_F(EkfAttitudeEstimatorTest, HeadingFusionOnNoMagnetometerMissingWarning)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Heading,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // Feed ONLY IMU samples during alignment (no magnetometer sample sent).
    feed_imu(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    EXPECT_TRUE(estimator.mag_missing());

    // Feed an IMU sample to inspect published message.
    feed_imu(1, level_accel, {});
    vk::msg::VehicleAttitude out{};
    drain_primary(out);

    EXPECT_TRUE(out.mag_missing);
    const auto e = Quaternion::from(out.q).euler();
    // 1e-3F absorbs floating-point roundoff in quaternion attitude initialization with yaw at zero.
    EXPECT_NEAR(e.yaw, 0.0F, 1e-3F);

    // Any magnetometer sample sent after alignment must NOT be fused because mag_missing is active.
    send_mag(Vector3{30.0F, 0.0F, 0.0F});
    feed_imu(1, level_accel, {});
    drain_primary(out);

    EXPECT_EQ(out.mag_fused, 0U);
    EXPECT_TRUE(out.mag_missing);
}

// 8. Restart: returns to aligning, and samples (IMU and magnetometer) taken before restart are dropped.
TEST_F(EkfAttitudeEstimatorTest, RestartsAlignmentAndDropsSamplesBeforeRestart)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Heading,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    const Vector3 east_field{0.0F, 30.0F, 0.0F};
    feed_both(201, level_accel, {}, east_field);
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    // Issue estimator restart.
    vk::core::Publication<vk::msg::EstimatorRestart> restart_pub{0};
    restart_pub.publish(vk::msg::EstimatorRestart{.timestamp = fake_now});

    // An early IMU sample and early mag sample taken before restart timestamp reach queue after it.
    vk::msg::ImuSample early_imu{};
    early_imu.timestamp = fake_now - period_us;
    early_imu.device_instance = imu_instance;
    early_imu.accel = level_accel.array();
    imu_pub.publish(early_imu);

    vk::msg::MagnetometerSample early_mag{};
    early_mag.timestamp = fake_now - period_us;
    early_mag.device_instance = mag_instance;
    early_mag.field = east_field.array();
    mag_pub.publish(early_mag);

    queue.process();
    EXPECT_EQ(estimator.state(), EkfAttitudeEstimator::State::Aligning);

    // 200 post-restart samples (995 ms) are not enough to align.
    const Vector3 north_field{30.0F, 0.0F, 0.0F};
    feed_both(200, level_accel, {}, north_field);
    EXPECT_EQ(estimator.state(), EkfAttitudeEstimator::State::Aligning);

    // 201st sample completes 1 s of stillness.
    feed_both(1, level_accel, {}, north_field);
    EXPECT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    vk::msg::VehicleAttitude out{};
    feed_both(5, level_accel, {}, north_field);
    drain_primary(out);

    const auto e = Quaternion::from(out.q).euler();
    // 1e-3F absorbs atan2 and levelling trigonometric precision.
    EXPECT_NEAR(e.yaw, 0.0F, 1e-3F);
}

// 9. Zero rate update: a vehicle sitting still with body velocity near zero fuses the zero
//    rate update and learns a z gyro bias.
TEST_F(EkfAttitudeEstimatorTest, ZeroRateUpdateStillVehicleLearnsZBias)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // 1 s alignment while still with zero body velocity.
    feed_imu_with_velocity(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    vk::msg::VehicleAttitude out{};
    drain_primary(out);

    // Now introduce a constant z gyro bias while still at rest.
    // 2000 samples = 10 s = 50 zero rate updates (every 0.2 s).
    const Vector3 gyro_with_z_bias{0.0F, 0.0F, 0.03F};
    feed_imu_with_velocity(2000, level_accel, gyro_with_z_bias, 0.0F, 0.0F, 0.0F);
    drain_primary(out);

    EXPECT_GE(out.zero_rate_fused, 45U);
    // The bias is constant in the model, so the estimate is close to the mean of the window averages.
    // The first window straddles the step and averages less than 0.03, which leaves an error of at most
    // 0.03 / 50 = 6e-4 rad/s; 1e-3F absorbs it.
    EXPECT_NEAR(out.gyro_bias[2], 0.03F, 1e-3F);
}

// 10. Zero rate update: a constant-speed straight drive (vx = 0.5 m/s, still IMU) fuses no zero rate update.
TEST_F(EkfAttitudeEstimatorTest, ZeroRateUpdateConstantSpeedStraightDriveFusesNone)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // 1 s alignment while still with zero velocity.
    feed_imu_with_velocity(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    vk::msg::VehicleAttitude out{};
    drain_primary(out);

    // Constant-speed straight drive: vx = 0.5 m/s (above 0.01 m/s stillness threshold),
    // specific force is gravity, gyro is zero.
    feed_imu_with_velocity(200, level_accel, {}, 0.5F, 0.0F, 0.0F);
    drain_primary(out);

    EXPECT_EQ(out.zero_rate_fused, 0U);
    EXPECT_FALSE(estimator.rest_detector().is_still());
    EXPECT_FALSE(estimator.rest_detector().at_rest());
}

// 11. Zero rate update: moving IMU (angular rate above threshold) fuses no zero rate update even with zero velocity.
TEST_F(EkfAttitudeEstimatorTest, ZeroRateUpdateMovingImuFusesNone)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // 1 s alignment while still.
    feed_imu_with_velocity(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    vk::msg::VehicleAttitude out{};
    drain_primary(out);

    // Moving IMU: gyro rate 0.2 rad/s > 0.1 rad/s still_gyro threshold, while wheels are stopped.
    const Vector3 moving_gyro{0.2F, 0.0F, 0.0F};
    feed_imu_with_velocity(200, level_accel, moving_gyro, 0.0F, 0.0F, 0.0F);
    drain_primary(out);

    EXPECT_EQ(out.zero_rate_fused, 0U);
    EXPECT_FALSE(estimator.rest_detector().is_still());
    EXPECT_FALSE(estimator.rest_detector().at_rest());
}

// 12. Zero rate update: without body velocity measurements published, no zero rate update is fused.
TEST_F(EkfAttitudeEstimatorTest, ZeroRateUpdateNoBodyVelocityFusesNone)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // Feed IMU samples ONLY (no body velocity published).
    feed_imu(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    feed_imu(200, level_accel, {});
    vk::msg::VehicleAttitude out{};
    drain_primary(out);

    EXPECT_EQ(out.zero_rate_fused, 0U);
    EXPECT_FALSE(estimator.rest_detector().at_rest());
}

// 13. Estimator restart: resets the rest detector and returns to aligning.
TEST_F(EkfAttitudeEstimatorTest, EstimatorRestartResetsRestDetector)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // Align and run while still with zero velocity.
    feed_imu_with_velocity(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    feed_imu_with_velocity(100, level_accel, {});
    EXPECT_TRUE(estimator.rest_detector().at_rest());

    // Issue estimator restart; the next IMU sample wakes the estimator, which handles it first.
    vk::core::Publication<vk::msg::EstimatorRestart> restart_pub{0};
    restart_pub.publish(vk::msg::EstimatorRestart{.timestamp = fake_now});
    feed_imu_with_velocity(1, level_accel, {});

    EXPECT_EQ(estimator.state(), EkfAttitudeEstimator::State::Aligning);
    // The detector starts over: the one sample since the restart is still, but not yet at rest.
    EXPECT_FALSE(estimator.rest_detector().at_rest());
    EXPECT_EQ(estimator.rest_detector().window_samples(), 0U);
}

// 14. Yaw variance without heading fusion: right after alignment, yaw variance is near zero (< 1e-5 rad^2).
TEST_F(EkfAttitudeEstimatorTest, YawVarianceWithoutHeadingFusionNearZeroAfterAlignment)
{
    EkfAttitudeEstimator::Config config{
        .imu_instance = imu_instance,
        .mag_instance = mag_instance,
        .primary = true,
        .mag_type = EkfMagType::Off,
    };
    EkfAttitudeEstimator estimator(config, queue);
    estimator.start();

    // 201 samples (1.005 s) completes alignment without heading fusion.
    feed_imu(201, level_accel, {});
    ASSERT_EQ(estimator.state(), EkfAttitudeEstimator::State::Running);

    // Feed one sample to trigger publication.
    feed_imu(1, level_accel, {});
    vk::msg::VehicleAttitude out{};
    drain_primary(out);

    // Yaw variance right after alignment is near zero (< 1e-5 rad^2), initialized to 1e-6 rad^2,
    // while roll and pitch start from the initial attitude variance and the gravity measurement.
    EXPECT_GT(out.variance[0], out.variance[2]);
    EXPECT_GT(out.variance[1], out.variance[2]);
    EXPECT_GT(out.variance[2], 0.0F);
    EXPECT_LT(out.variance[2], 1e-5F);
    // 1e-7F absorbs continuous-time gyro noise covariance propagation over a single 5 ms sample step.
    EXPECT_NEAR(out.variance[2], 1e-6F, 1e-7F);
}

}  // namespace
