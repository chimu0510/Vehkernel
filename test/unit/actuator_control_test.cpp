// Unit tests for actuator control (ADR-0013, spec 0006): the wheel speed PI
// against the simulated motor, its clamps and anti-windup, NaN and disabled
// outputs, the reversal settings and 16-bit encoder wrap.
#include <vk/actuators/actuator_control.hpp>
#include <vk/sim/motor.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>

namespace {

using vk::actuators::ActuatorControl;
using vk::actuators::Settings;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

constexpr std::uint64_t cycle_us = 5'000;
constexpr float dt = 0.005F;
constexpr float nan = std::numeric_limits<float>::quiet_NaN();

// One output closed through the simulated motor, cycle by cycle.
// The fake clock only moves forward, so a motor made for one test never
// sees time run backwards.
class WheelLoop : public ::testing::Test {
protected:
    void run(float target, std::uint64_t duration_us, bool enabled = true)
    {
        for (std::uint64_t t = 0; t < duration_us; t += cycle_us) {
            fake_now += cycle_us;
            const std::array<std::uint16_t, 1> counts{motor.count()};
            const std::array<float, 1> targets{target};
            control.step(dt, counts, targets, enabled, settings, applied);
            applied[0] = control.output(0).duty;
            motor.drive(applied[0]);
            largest_change = std::max(largest_change, std::fabs(applied[0] - last));
            last = applied[0];
        }
    }

    std::array<float, 1> applied{};
    float last = 0.0F;
    float largest_change = 0.0F;

    vk::sim::Motor motor{{}, &fake_clock};
    ActuatorControl control{1};
    Settings settings{};
};

TEST_F(WheelLoop, ReachesAndHoldsTheTarget)
{
    run(10.0F, 1'000'000);
    EXPECT_NEAR(control.output(0).speed, 10.0F, 0.5F);
    EXPECT_NEAR(motor.speed(), 10.0F, 0.5F);
    EXPECT_FLOAT_EQ(control.output(0).target, 10.0F);
}

TEST_F(WheelLoop, HoldsZeroSpeedAtAZeroTarget)
{
    run(10.0F, 500'000);
    run(0.0F, 1'000'000);
    EXPECT_NEAR(motor.speed(), 0.0F, 0.2F);
}

TEST_F(WheelLoop, ClampsTheTargetToTheMaximumSpeed)
{
    settings.loop.max_speed = 5.0F;
    run(20.0F, 1'000'000);
    EXPECT_FLOAT_EQ(control.output(0).target, 5.0F);
    EXPECT_NEAR(motor.speed(), 5.0F, 0.3F);
}

TEST_F(WheelLoop, ClampsTheDutyAndDoesNotWindUp)
{
    // 0.1 of full duty reaches 5 rad/s at most, far from the 20 asked.
    settings.loop.out_max = 0.1F;
    run(20.0F, 2'000'000);
    EXPECT_FLOAT_EQ(control.output(0).duty, 0.1F);
    // Once the target is reachable again, it settles without a long overshoot.
    run(2.0F, 600'000);
    EXPECT_NEAR(motor.speed(), 2.0F, 0.3F);
}

TEST_F(WheelLoop, NaNLeavesTheOutputUncontrolledAndClearsTheIntegral)
{
    run(10.0F, 500'000);
    run(nan, cycle_us);
    EXPECT_FLOAT_EQ(control.output(0).duty, 0.0F);
    EXPECT_TRUE(std::isnan(control.output(0).target));
    // Back in control, the first duty holds at most one step of integral.
    settings.loop.ff = 0.0F;
    settings.loop.p = 0.0F;
    run(10.0F, cycle_us);
    EXPECT_LE(std::fabs(control.output(0).duty), settings.loop.i * 10.0F * dt + 1e-4F);
}

TEST_F(WheelLoop, DisabledOutputsGiveNoDuty)
{
    run(10.0F, 500'000, false);
    EXPECT_FLOAT_EQ(control.output(0).duty, 0.0F);
    EXPECT_NEAR(motor.speed(), 0.0F, 1e-3F);
}

TEST_F(WheelLoop, ReversalsFlipTheDutyAndTheEncoder)
{
    settings.outputs[0].reversed = true;
    run(10.0F, cycle_us);
    EXPECT_LT(control.output(0).duty, 0.0F);

    ActuatorControl encoder_reversed{1};
    Settings flipped{};
    flipped.outputs[0].encoder_reversed = true;
    const std::array<float, 1> none{nan};
    encoder_reversed.step(dt, std::array<std::uint16_t, 1>{100}, none, true, flipped);
    encoder_reversed.step(dt, std::array<std::uint16_t, 1>{110}, none, true, flipped);
    EXPECT_LT(encoder_reversed.output(0).speed, 0.0F);
}

TEST(ActuatorControl, SixteenBitCountsWrapInBothDirections)
{
    ActuatorControl control{2};
    Settings settings{};
    const std::array<float, 2> none{nan, nan};
    control.step(dt, std::array<std::uint16_t, 2>{65530, 5}, none, false, settings);
    control.step(dt, std::array<std::uint16_t, 2>{4, 65531}, none, false, settings);  // +10 and -10 counts
    const float expected = 10.0F * 2.0F * std::numbers::pi_v<float> / settings.outputs[0].counts_per_turn / dt;
    EXPECT_NEAR(control.output(0).speed, expected, 1e-3F);
    EXPECT_NEAR(control.output(1).speed, -expected, 1e-3F);
}

TEST(SimMotor, StopsWithoutPowerWhateverTheDuty)
{
    vk::sim::Motor motor{{}, &fake_clock};
    motor.drive(0.5F);
    fake_now += 1'000'000;
    EXPECT_NEAR(motor.speed(), 25.0F, 0.1F);
    motor.set_powered(false);
    fake_now += 1'000'000;
    EXPECT_NEAR(motor.speed(), 0.0F, 0.01F);
}

TEST_F(WheelLoop, ASetpointStepRampsNoFasterThanTheSlewRate)
{
    settings.loop.slew_rate = 0.5F;
    run(10.0F, 100'000);
    EXPECT_LE(largest_change, 0.5F * dt + 1e-6F);
    EXPECT_NEAR(control.output(0).duty, 0.5F * 0.1F, 1e-3F);  // still ramping after 0.1 s
    run(10.0F, 2'000'000);
    EXPECT_NEAR(motor.speed(), 10.0F, 0.5F);  // and no windup while it ramped
}

TEST_F(WheelLoop, ASlewRateOfZeroDoesNotLimit)
{
    settings.loop.slew_rate = 0.0F;
    run(10.0F, cycle_us);
    EXPECT_FLOAT_EQ(control.output(0).duty, settings.loop.out_max);
}

TEST_F(WheelLoop, StoppingIsNeverSlewLimited)
{
    settings.loop.slew_rate = 0.1F;
    run(10.0F, 3'000'000);
    ASSERT_GT(control.output(0).duty, 0.1F);
    run(nan, cycle_us);
    EXPECT_FLOAT_EQ(control.output(0).duty, 0.0F);
    run(10.0F, 3'000'000);
    run(10.0F, cycle_us, false);  // disarmed or the emergency stop
    EXPECT_FLOAT_EQ(control.output(0).duty, 0.0F);
}

TEST(OutputStage, SlewsOpenLoopDutiesButLetsAStopThrough)
{
    EXPECT_FLOAT_EQ(vk::actuators::slew(0.0F, 0.2F, 2.0F, 0.005F), 0.01F);
    EXPECT_FLOAT_EQ(vk::actuators::slew(0.2F, 0.0F, 2.0F, 0.005F), 0.0F);
    EXPECT_FLOAT_EQ(vk::actuators::slew(0.0F, 0.2F, 0.0F, 0.005F), 0.2F);
}

}  // namespace
