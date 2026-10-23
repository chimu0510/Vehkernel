// SPDX-License-Identifier: GPL-3.0-or-later
// The Mahony filter against synthetic motion with a known true attitude
// (ADR-0014). The IMU is simulated from the truth: the gyro reads the true
// body rate plus a bias, the accelerometer the specific force of the true
// attitude plus any linear acceleration.
#include <vk/estimation/filters/mahony.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <numbers>

namespace {

using vk::estimation::AccelerometerModel;
using vk::estimation::Euler;
using vk::estimation::MahonyFilter;
using vk::estimation::Quaternion;
using vk::estimation::Vector3;

constexpr float pi = std::numbers::pi_v<float>;
constexpr std::uint64_t period_us = 5'000;  // 200 Hz

float deg(float degrees)
{
    return degrees * pi / 180.0F;
}

float to_deg(float radians)
{
    return radians * 180.0F / pi;
}

// What the IMU feels at one instant.
struct Motion {
    Vector3 rate;                 // true body rate [rad/s]
    Vector3 linear_acceleration;  // in the body frame [m/s^2]
};

class Simulation {
public:
    Simulation(const Quaternion& truth, const Vector3& gyro_bias) : truth_(truth), gyro_bias_(gyro_bias) {}

    // Runs for `seconds`, the motion given per sample, with each step's
    // length from `step_us` (constant 5 ms by default).
    void run(MahonyFilter& filter, float seconds, const std::function<Motion(float)>& motion,
             const std::function<std::uint64_t(int)>& step_us = [](int) { return period_us; })
    {
        const auto end = time_us_ + static_cast<std::uint64_t>(seconds * 1e6F);
        for (int i = 0; time_us_ < end; ++i) {
            const Motion m = motion(static_cast<float>(time_us_) * 1e-6F);
            const Vector3 accel = AccelerometerModel::predict(truth_) + m.linear_acceleration;
            filter.update(m.rate + gyro_bias_, accel, time_us_);
            if (settled_) {
                max_error_after_ = std::max(max_error_after_, error(filter));
            }
            const std::uint64_t step = step_us(i);
            truth_ = truth_.integrated(m.rate, static_cast<float>(step) * 1e-6F);
            time_us_ += step;
        }
    }

    // Degrees between the truth now and the estimate.
    [[nodiscard]] float error(const MahonyFilter& filter) const { return to_deg(filter.attitude().angle_to(truth_)); }

    // From now on, record the largest error.
    void settle() { settled_ = true; }
    [[nodiscard]] float max_error() const { return max_error_after_; }

private:
    Quaternion truth_;
    Vector3 gyro_bias_;
    std::uint64_t time_us_ = 1'000'000;
    bool settled_ = false;
    float max_error_after_ = 0.0F;
};

Motion still(float /*t*/)
{
    return {};
}

TEST(MahonyFilter, AStaticTiltConvergesFromLevel)
{
    const Quaternion tilted = Quaternion::from_euler({.roll = deg(20), .pitch = deg(-15)});
    Simulation sim(tilted, {});
    MahonyFilter filter;
    filter.reset(Quaternion::identity());
    sim.run(filter, 5.0F, still);
    const Euler e = filter.attitude().euler();
    EXPECT_NEAR(to_deg(e.roll), 20.0F, 1.0F);
    EXPECT_NEAR(to_deg(e.pitch), -15.0F, 1.0F);
}

TEST(MahonyFilter, ResetFromTheAccelerometerStartsLevelledAtRest)
{
    const Quaternion tilted = Quaternion::from_euler({.roll = deg(-30), .pitch = deg(25), .yaw = deg(80)});
    MahonyFilter filter;
    filter.reset_from(AccelerometerModel::predict(tilted));
    const Euler e = filter.attitude().euler();
    EXPECT_NEAR(to_deg(e.roll), -30.0F, 0.01F);
    EXPECT_NEAR(to_deg(e.pitch), 25.0F, 0.01F);
    EXPECT_NEAR(e.yaw, 0.0F, 1e-6F);
}

TEST(MahonyFilter, TracksAConstantRotationAboutEachAxis)
{
    for (const Vector3& rate : {Vector3{deg(45), 0.0F, 0.0F}, Vector3{0.0F, deg(45), 0.0F}, Vector3{0.0F, 0.0F, deg(45)}}) {
        const Quaternion start = Quaternion::from_euler({.roll = deg(5), .pitch = deg(-5)});
        Simulation sim(start, {});
        MahonyFilter filter;
        filter.reset(start);
        sim.settle();
        sim.run(filter, 4.0F, [&](float) { return Motion{.rate = rate, .linear_acceleration = {}}; });
        EXPECT_LT(sim.max_error(), 2.0F) << "rate " << rate.x << " " << rate.y << " " << rate.z;
    }
}

TEST(MahonyFilter, EstimatesTheGyroBiasOnRollAndPitchAxes)
{
    const Vector3 bias{0.02F, -0.015F, 0.0F};
    Simulation sim(Quaternion::identity(), bias);
    MahonyFilter filter;
    filter.reset(Quaternion::identity());
    sim.run(filter, 120.0F, still);
    EXPECT_NEAR(filter.gyro_bias().x, bias.x, 0.005F);
    EXPECT_NEAR(filter.gyro_bias().y, bias.y, 0.005F);
    EXPECT_LT(sim.error(filter), 0.5F);
    // The bias-corrected rate is near zero at rest.
    EXPECT_NEAR(filter.angular_velocity().x, 0.0F, 0.005F);
}

TEST(MahonyFilter, RejectsLinearAccelerationBursts)
{
    const Quaternion start = Quaternion::from_euler({.roll = deg(10)});
    Simulation sim(start, {});
    MahonyFilter filter;
    filter.reset(start);
    sim.settle();
    // 1 s bursts of 6 m/s^2 forward every 3 s: far enough from gravity to reject.
    sim.run(filter, 15.0F, [](float t) {
        const bool burst = static_cast<int>(t) % 3 == 0;
        return Motion{.rate = {}, .linear_acceleration = burst ? Vector3{6.0F, 0.0F, 0.0F} : Vector3{}};
    });
    EXPECT_LT(sim.max_error(), 2.0F);
    EXPECT_GT(filter.accel_rejected(), 900U);
}

TEST(MahonyFilter, HandlesUnevenTimeSteps)
{
    // Steps of 0 to 10 ms, a full sample period of jitter either way.
    const auto jittered = [](int i) { return static_cast<std::uint64_t>((i * 7919) % 11) * 1'000; };
    const Quaternion start = Quaternion::from_euler({.pitch = deg(10)});
    Simulation sim(start, {});
    MahonyFilter filter;
    filter.reset(start);
    sim.settle();
    sim.run(filter, 4.0F, [](float) { return Motion{.rate = {deg(30), deg(-20), deg(45)}, .linear_acceleration = {}}; }, jittered);
    EXPECT_LT(sim.max_error(), 2.0F);
}

TEST(MahonyFilter, AnOutOfOrderSampleOnlyResetsTheTime)
{
    MahonyFilter filter;
    filter.reset(Quaternion::identity());
    const Vector3 up{0.0F, 0.0F, -9.80665F};
    filter.update({1.0F, 0.0F, 0.0F}, up, 2'000'000);
    filter.update({1.0F, 0.0F, 0.0F}, up, 1'000'000);  // older: no step
    EXPECT_NEAR(filter.attitude().angle_to(Quaternion::identity()), 0.0F, 1e-6F);
    filter.update({1.0F, 0.0F, 0.0F}, up, 1'005'000);
    EXPECT_GT(filter.attitude().angle_to(Quaternion::identity()), 0.004F);
}

}  // namespace
