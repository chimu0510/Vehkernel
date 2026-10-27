// SPDX-License-Identifier: GPL-3.0-or-later
// Local odometry (ADR-0014): the dead-reckoning step against closed forms,
// the covariance it grows, and the estimator work item on a fake clock,
// which takes its heading from vehicle_attitude.
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/estimation/local_odometry.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/models/dead_reckoning.hpp>
#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_local_odometry.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace {

using vk::estimation::DeadReckoning;
using vk::estimation::LocalOdometry;
using vk::estimation::Quaternion;
using vk::estimation::WheelGeometry;
using vk::estimation::WheelKinematics;

constexpr float pi = std::numbers::pi_v<float>;
constexpr float dt = 0.005F;

// Steps `seconds` at a constant body velocity, the heading turning at `yaw_rate`.
void drive(DeadReckoning& d, float vx, float vy, float yaw_rate, float seconds, float& heading,
           float var_v = 1e-4F, float var_heading = 0.0F)
{
    const auto steps = static_cast<int>(std::lround(seconds / dt));
    for (int i = 0; i < steps; ++i) {
        heading = std::remainder(heading + yaw_rate * dt, 2.0F * pi);
        d.step(dt, vx, vy, var_v, var_v, heading, var_heading);
    }
}

TEST(DeadReckoning, StraightAlongTheHeading)
{
    DeadReckoning d;
    float heading = pi / 2;  // east
    d.step(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, heading, 0.0F);
    drive(d, 0.5F, 0.0F, 0.0F, 2.0F, heading);
    EXPECT_NEAR(d.north(), 0.0, 1e-5);
    EXPECT_NEAR(d.east(), 1.0, 1e-5);
    EXPECT_NEAR(d.distance(), 1.0F, 1e-4F);
    // vy moves right of the heading: south, facing east.
    drive(d, 0.0F, 0.25F, 0.0F, 2.0F, heading);
    EXPECT_NEAR(d.north(), -0.5, 1e-5);
}

TEST(DeadReckoning, TurningFollowsTheArc)
{
    DeadReckoning d;
    float heading = 0.0F;
    d.step(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, heading, 0.0F);
    // 0.5 m/s at 0.5 rad/s: a quarter circle of radius 1 m to the right.
    drive(d, 0.5F, 0.0F, 0.5F, pi, heading);
    EXPECT_NEAR(heading, pi / 2, 1e-3F);
    EXPECT_NEAR(d.north(), 1.0, 2e-3);
    EXPECT_NEAR(d.east(), 1.0, 2e-3);
}

TEST(DeadReckoning, StationaryStaysPut)
{
    DeadReckoning d;
    float heading = 1.0F;
    d.step(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, heading, 0.0F);
    // Turning in place moves nothing.
    drive(d, 0.0F, 0.0F, 1.0F, 2.0F, heading, 0.0F, 1e-3F);
    EXPECT_EQ(d.north(), 0.0);
    EXPECT_EQ(d.east(), 0.0);
    EXPECT_FLOAT_EQ(d.covariance()[0], 0.0F);
    EXPECT_FLOAT_EQ(d.covariance()[2], 0.0F);
}

TEST(DeadReckoning, CovarianceGrowsWithDistance)
{
    DeadReckoning d;
    float heading = 0.0F;
    d.step(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, heading, 0.0F);
    drive(d, 0.5F, 0.0F, 0.0F, 1.0F, heading, 1e-4F, 1e-3F);
    const auto one = d.covariance();
    drive(d, 0.5F, 0.0F, 0.0F, 1.0F, heading, 1e-4F, 1e-3F);
    const auto two = d.covariance();
    EXPECT_GT(one[0], 0.0F);
    EXPECT_GT(two[0], one[0]);
    // Driving north, the heading's uncertainty spreads the position east.
    EXPECT_GT(two[2], one[2]);
    EXPECT_GT(two[2], two[0]);
}

TEST(DeadReckoning, CovarianceGrowsWithTheResidual)
{
    // Two skid-steer runs over the same distance, one with a slipping wheel.
    const std::array<WheelGeometry, 4> wheels{{{.y = -0.1F, .radius = 0.05F},
                                               {.y = 0.1F, .radius = 0.05F},
                                               {.y = -0.1F, .radius = 0.05F},
                                               {.y = 0.1F, .radius = 0.05F}}};
    const WheelKinematics k(wheels);
    const std::array<float, 4> noise{0.02F, 0.02F, 0.02F, 0.02F};
    const auto clean = k.measure(std::array<float, 4>{10.0F, 10.0F, 10.0F, 10.0F}, noise);
    const auto slipping = k.measure(std::array<float, 4>{12.0F, 10.0F, 10.0F, 10.0F}, noise);
    EXPECT_GT(slipping.residual, clean.residual);
    EXPECT_GT(slipping.variance[0], clean.variance[0]);
    EXPECT_GT(slipping.variance[2], clean.variance[2]);
    // Only the noise when the wheels agree: vx's variance is r² noise² / 4.
    EXPECT_NEAR(clean.variance[0], 0.05F * 0.05F * 0.02F * 0.02F / 4.0F, 1e-10F);

    DeadReckoning a;
    DeadReckoning b;
    for (int i = 0; i < 400; ++i) {
        a.step(dt, clean.twist.vx, 0.0F, clean.variance[0], 0.0F, 0.0F, 0.0F);
        b.step(dt, slipping.twist.vx, 0.0F, slipping.variance[0], 0.0F, 0.0F, 0.0F);
    }
    EXPECT_GT(b.covariance()[0], a.covariance()[0]);
}

TEST(DeadReckoning, ResetClearsPositionAndCovariance)
{
    DeadReckoning d;
    float heading = 0.0F;
    d.step(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, heading, 0.0F);
    drive(d, 0.5F, 0.0F, 0.0F, 2.0F, heading);
    EXPECT_GT(d.distance(), 0.0F);
    EXPECT_GT(d.north(), 0.0);
    d.reset();
    EXPECT_EQ(d.north(), 0.0);
    EXPECT_EQ(d.east(), 0.0);
    EXPECT_FLOAT_EQ(d.distance(), 0.0F);
    EXPECT_FLOAT_EQ(d.covariance()[0], 0.0F);
    EXPECT_FLOAT_EQ(d.covariance()[1], 0.0F);
    EXPECT_FLOAT_EQ(d.covariance()[2], 0.0F);
}

// The estimator work item.
std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

constexpr std::uint8_t instance = 1;

class LocalOdometryTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        estimator.start();
        vk::msg::VehicleLocalOdometry stale{};
        while (odometry.copy(stale)) {
        }
    }

    void publish_attitude(float yaw)
    {
        vk::msg::VehicleAttitude a{};
        a.timestamp = fake_now;
        a.q = Quaternion::from_euler({.yaw = yaw}).array();
        a.angular_velocity = {0.0F, 0.0F, 0.25F};
        a.variance = {1e-4F, 1e-4F, 2e-3F};
        attitude.publish(a);
    }

    // One measurement 5 ms after the last, then a run of the queue.
    void measure(float vx, float vy)
    {
        fake_now += 5'000;
        vk::msg::BodyVelocityMeasurement m{};
        m.timestamp = fake_now;
        m.timestamp_sample = fake_now;
        m.vx = vx;
        m.vy = vy;
        m.variance = {1e-4F, 0.0F, 1e-3F};
        measurement.publish(m);
        queue.process();
    }

    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    vk::core::Publication<vk::msg::VehicleAttitude> attitude{0};
    vk::core::Publication<vk::msg::BodyVelocityMeasurement> measurement{instance};
    vk::core::Subscription<vk::msg::VehicleLocalOdometry> odometry{0};
    LocalOdometry estimator{{.measurement = instance}, queue};
};

TEST_F(LocalOdometryTest, DeadReckonsAlongTheAttitudesHeading)
{
    publish_attitude(pi / 2);  // facing east
    for (int i = 0; i < 201; ++i) {
        measure(0.5F, 0.0F);
        publish_attitude(pi / 2);
    }
    vk::msg::VehicleLocalOdometry out{};
    ASSERT_TRUE(odometry.copy(out));
    // 200 steps of 5 ms after the first, which only sets the origin.
    EXPECT_NEAR(out.position[0], 0.0F, 1e-4F);
    EXPECT_NEAR(out.position[1], 0.5F, 1e-4F);
    EXPECT_FLOAT_EQ(out.position[2], 0.0F);
    EXPECT_EQ(out.estimator_id, LocalOdometry::estimator_id);
    EXPECT_FLOAT_EQ(out.velocity[0], 0.5F);
    EXPECT_FLOAT_EQ(out.yaw_rate, 0.25F);  // the gyro's
    EXPECT_FLOAT_EQ(out.heading_variance, 2e-3F);
    EXPECT_FLOAT_EQ(out.velocity_variance[2], 1e-3F);
    EXPECT_GT(out.position_variance[2], 0.0F);
    EXPECT_EQ(out.timestamp_sample, fake_now);
    EXPECT_NEAR(Quaternion::from(out.q).euler().yaw, pi / 2, 1e-5F);
}

TEST_F(LocalOdometryTest, WaitsForAHeading)
{
    // No attitude for longer than the timeout: nothing is published, nothing moves.
    fake_now += 2 * LocalOdometry::attitude_timeout_us;
    for (int i = 0; i < 20; ++i) {
        measure(1.0F, 0.0F);
    }
    vk::msg::VehicleLocalOdometry out{};
    EXPECT_FALSE(odometry.copy(out));
    publish_attitude(0.0F);
    measure(1.0F, 0.0F);
    ASSERT_TRUE(odometry.copy(out));
    // The first step with a heading moves by its own 5 ms only.
    EXPECT_NEAR(out.position[0], 0.005F, 1e-6F);
}

TEST_F(LocalOdometryTest, RestartsFromZeroOnEstimatorRestart)
{
    publish_attitude(pi / 2);  // facing east
    for (int i = 0; i < 201; ++i) {
        measure(0.5F, 0.0F);
        publish_attitude(pi / 2);
    }
    vk::msg::VehicleLocalOdometry out{};
    ASSERT_TRUE(odometry.copy(out));
    EXPECT_NEAR(out.position[1], 0.5F, 1e-4F);

    // Estimator restart arrives.
    vk::core::Publication<vk::msg::EstimatorRestart> restart;
    restart.publish(vk::msg::EstimatorRestart{.timestamp = fake_now});

    // While attitude is not available (aligning), no odometry is published.
    for (int i = 0; i < 20; ++i) {
        measure(0.5F, 0.0F);
    }
    EXPECT_FALSE(odometry.copy(out));

    // Attitude estimator has aligned and publishes again.
    publish_attitude(0.0F);  // facing north now
    measure(0.5F, 0.0F);
    ASSERT_TRUE(odometry.copy(out));
    // Restarts from zero: moves by only the 5 ms step.
    EXPECT_NEAR(out.position[0], 0.0025F, 1e-5F);
    EXPECT_NEAR(out.position[1], 0.0F, 1e-5F);
}

}  // namespace
