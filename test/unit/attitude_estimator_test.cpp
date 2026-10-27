// SPDX-License-Identifier: GPL-3.0-or-later
// The attitude estimator work item on a fake clock (ADR-0008, ADR-0014):
// alignment, draining the IMU queue, what it publishes, and the ATT_*
// parameters (ADR-0012).
#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/estimation/attitude_estimator.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/params/params.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <numbers>
#include <string_view>

namespace {

using vk::estimation::AccelerometerModel;
using vk::estimation::AttitudeEstimator;
using vk::estimation::Quaternion;
using vk::estimation::Vector3;

constexpr float pi = std::numbers::pi_v<float>;
constexpr std::uint8_t imu_instance = 1;
constexpr std::uint64_t period_us = 5'000;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

class AttitudeEstimatorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        // Drop what earlier tests left on vehicle_attitude.
        vk::msg::VehicleAttitude stale{};
        drain(stale);
    }

    // Leaves the ATT_* parameters at their defaults for the next test.
    void TearDown() override
    {
        auto& parameters = vk::params::parameters();
        for (std::size_t i = 0; i < parameters.count(); ++i) {
            parameters.set(i, parameters.info(i).default_value);
        }
    }

    static void set(std::string_view name, float value)
    {
        ASSERT_EQ(vk::params::parameters().set(name, vk::core::ParamValue::from_float(value)),
                  vk::core::ParamResult::Ok);
    }

    // Publishes `count` samples 5 ms apart, `per_run` of them before each run of the queue.
    void feed(int count, const Vector3& accel, const Vector3& gyro, int per_run = 1)
    {
        for (int i = 0; i < count; ++i) {
            vk::msg::ImuSample sample{};
            sample.timestamp = fake_now;
            sample.device_instance = imu_instance;
            sample.accel = accel.array();
            sample.gyro = gyro.array();
            imu.publish(sample);
            fake_now += period_us;
            if ((i + 1) % per_run == 0) {
                queue.process();
            }
        }
        queue.process();
    }

    // vehicle_attitude publications since the last call; `last` gets the newest.
    int drain(vk::msg::VehicleAttitude& last)
    {
        const std::uint64_t generation = vk::core::topic<vk::msg::VehicleAttitude>(0).generation();
        const auto count = static_cast<int>(generation - seen_);
        seen_ = generation;
        while (attitude.copy(last)) {
        }
        return count;
    }

    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    vk::core::Publication<vk::msg::ImuSample> imu{imu_instance};
    vk::core::Subscription<vk::msg::VehicleAttitude> attitude;
    AttitudeEstimator::Config config{.imu_instance = imu_instance};
    std::uint64_t seen_ = 0;
};

const Vector3 level = AccelerometerModel::predict(Quaternion::identity());

TEST_F(AttitudeEstimatorTest, PublishesNothingWhileAligning)
{
    AttitudeEstimator estimator(config, queue);
    estimator.start();
    feed(100, level, {});  // 0.5 s still, half the alignment time
    vk::msg::VehicleAttitude out{};
    EXPECT_EQ(drain(out), 0);
    EXPECT_EQ(estimator.state(), AttitudeEstimator::State::Aligning);
}

TEST_F(AttitudeEstimatorTest, AlignsRollPitchAndGyroBiasFromStillSamples)
{
    const Quaternion tilted = Quaternion::from_euler({.roll = 0.3F, .pitch = -0.2F, .yaw = 1.0F});
    const Vector3 bias{0.01F, -0.02F, 0.005F};
    AttitudeEstimator estimator(config, queue);
    estimator.start();
    feed(201, AccelerometerModel::predict(tilted), bias);  // just over 1 s
    ASSERT_EQ(estimator.state(), AttitudeEstimator::State::Running);
    feed(10, AccelerometerModel::predict(tilted), bias);

    vk::msg::VehicleAttitude out{};
    EXPECT_EQ(drain(out), 10);
    const auto e = Quaternion::from(out.q).euler();
    EXPECT_NEAR(e.roll, 0.3F, 1e-3F);
    EXPECT_NEAR(e.pitch, -0.2F, 1e-3F);
    EXPECT_NEAR(e.yaw, 0.0F, 1e-3F);  // yaw starts at 0
    EXPECT_NEAR(out.gyro_bias[0], bias.x, 1e-3F);
    EXPECT_NEAR(out.gyro_bias[1], bias.y, 1e-3F);
    EXPECT_NEAR(out.gyro_bias[2], bias.z, 1e-3F);
    EXPECT_NEAR(out.angular_velocity[0], 0.0F, 1e-3F);
    EXPECT_EQ(out.estimator_id, AttitudeEstimator::estimator_id);
    EXPECT_EQ(out.timestamp, fake_now);
    EXPECT_EQ(out.timestamp_sample, fake_now - period_us);
    EXPECT_FLOAT_EQ(out.variance[0], config.level_variance);
    EXPECT_GT(out.variance[2], 0.0F);
    EXPECT_EQ(out.accel_rejected, 0U);
}

TEST_F(AttitudeEstimatorTest, MovementRestartsAlignment)
{
    AttitudeEstimator estimator(config, queue);
    estimator.start();
    feed(150, level, {});                      // 0.75 s still
    feed(1, level, {0.0F, 0.0F, pi / 2.0F});   // a turn
    feed(150, level, {});                      // 0.75 s still again: not enough
    EXPECT_EQ(estimator.state(), AttitudeEstimator::State::Aligning);
    feed(60, level, {});                       // past 1 s since the turn
    EXPECT_EQ(estimator.state(), AttitudeEstimator::State::Running);
}

TEST_F(AttitudeEstimatorTest, ConsumesEveryQueuedSample)
{
    AttitudeEstimator estimator(config, queue);
    estimator.start();
    feed(201, level, {});
    vk::msg::VehicleAttitude out{};
    drain(out);
    // Three samples per run, as when wq:ctrl runs late: each one is estimated.
    feed(30, level, {}, 3);
    EXPECT_EQ(drain(out), 30);
}

TEST_F(AttitudeEstimatorTest, AppliesNewGainsFromTheNextSample)
{
    AttitudeEstimator estimator(config, queue);
    estimator.start();
    feed(201, level, {});
    ASSERT_EQ(estimator.state(), AttitudeEstimator::State::Running);

    // Without gains, a tilt the gyro never saw is not corrected.
    set("ATT_KP", 0.0F);
    set("ATT_KI", 0.0F);
    const Vector3 tilted = AccelerometerModel::predict(Quaternion::from_euler({.roll = 0.2F, .pitch = 0.0F, .yaw = 0.0F}));
    feed(200, tilted, {});
    vk::msg::VehicleAttitude out{};
    drain(out);
    EXPECT_NEAR(Quaternion::from(out.q).euler().roll, 0.0F, 1e-4F);

    // A large gain pulls roll to the accelerometer within a second.
    set("ATT_KP", 5.0F);
    feed(200, tilted, {});
    drain(out);
    EXPECT_NEAR(Quaternion::from(out.q).euler().roll, 0.2F, 0.005F);
}

TEST_F(AttitudeEstimatorTest, RestartsAlignmentOnEstimatorRestart)
{
    const Quaternion tilted = Quaternion::from_euler({.roll = 0.3F, .pitch = -0.2F, .yaw = 1.0F});
    const Vector3 bias{0.01F, -0.02F, 0.005F};
    AttitudeEstimator estimator(config, queue);
    estimator.start();
    feed(201, AccelerometerModel::predict(tilted), bias);
    ASSERT_EQ(estimator.state(), AttitudeEstimator::State::Running);
    feed(10, AccelerometerModel::predict(tilted), bias);
    vk::msg::VehicleAttitude out{};
    EXPECT_EQ(drain(out), 10);

    // Restart the estimators.
    vk::core::Publication<vk::msg::EstimatorRestart> restart;
    restart.publish(vk::msg::EstimatorRestart{.timestamp = fake_now});

    // A different tilt and bias for the new alignment.
    const Quaternion new_tilted = Quaternion::from_euler({.roll = -0.15F, .pitch = 0.25F, .yaw = 0.5F});
    const Vector3 new_bias{-0.02F, 0.01F, -0.01F};

    // 0.5 s still: publishes nothing, stays in Aligning.
    feed(100, AccelerometerModel::predict(new_tilted), new_bias);
    EXPECT_EQ(drain(out), 0);
    EXPECT_EQ(estimator.state(), AttitudeEstimator::State::Aligning);

    // Another 101 samples (total > 1 s still): finishes alignment.
    feed(101, AccelerometerModel::predict(new_tilted), new_bias);
    ASSERT_EQ(estimator.state(), AttitudeEstimator::State::Running);

    // Now it runs and publishes from the new samples.
    feed(10, AccelerometerModel::predict(new_tilted), new_bias);
    EXPECT_EQ(drain(out), 10);
    const auto e = Quaternion::from(out.q).euler();
    EXPECT_NEAR(e.roll, -0.15F, 1e-3F);
    EXPECT_NEAR(e.pitch, 0.25F, 1e-3F);
    EXPECT_NEAR(e.yaw, 0.0F, 1e-3F);  // yaw starts at 0
    EXPECT_NEAR(out.gyro_bias[0], new_bias.x, 1e-3F);
    EXPECT_NEAR(out.gyro_bias[1], new_bias.y, 1e-3F);
    EXPECT_NEAR(out.gyro_bias[2], new_bias.z, 1e-3F);
}

TEST_F(AttitudeEstimatorTest, IgnoresSamplesTakenBeforeTheRestart)
{
    AttitudeEstimator estimator(config, queue);
    estimator.start();

    // A sample taken just before the restart reaches the queue after it, as on the board, where a
    // sample is published after the time it was taken at.
    vk::msg::ImuSample early{};
    early.timestamp = fake_now;
    early.device_instance = imu_instance;
    early.accel = level.array();
    imu.publish(early);
    fake_now += period_us / 2;
    vk::core::Publication<vk::msg::EstimatorRestart> restart;
    restart.publish(vk::msg::EstimatorRestart{.timestamp = fake_now});
    fake_now += period_us / 2;
    queue.process();

    // The alignment needs 1 s from the first sample taken after the restart: the early one does not
    // count, so 200 samples (995 ms after the first of them) are not enough and the 201st is.
    feed(200, level, {});
    EXPECT_EQ(estimator.state(), AttitudeEstimator::State::Aligning);
    feed(1, level, {});
    EXPECT_EQ(estimator.state(), AttitudeEstimator::State::Running);
}

}  // namespace
