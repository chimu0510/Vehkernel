// Unit tests for the motor-to-encoder bring-up case (ADR-0018, spec 0006)
// against four simulated motors, with each wiring fault it must find.
#include <vk/actuators/bringup_case.hpp>
#include <vk/sim/motor.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace {

using vk::actuators::BringupCase;
using vk::actuators::BringupResult;
using vk::actuators::BringupState;
using vk::actuators::OutputResult;
using vk::actuators::Settings;

std::uint64_t fake_now = 1'000'000;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

constexpr std::uint64_t cycle_us = 5'000;

class BringupTest : public ::testing::Test {
protected:
    BringupTest()
    {
        for (auto& motor : motors) {
            motor = std::make_unique<vk::sim::Motor>(vk::sim::Motor::Config{}, &fake_clock);
        }
    }

    std::array<std::uint16_t, 4> counts()
    {
        std::array<std::uint16_t, 4> out{};
        for (std::size_t i = 0; i < 4; ++i) {
            out[i] = motors[encoder_of[i]]->count();
        }
        return out;
    }

    // Runs the whole case, or `duration_us` of it.
    void run(std::uint64_t duration_us = 10'000'000)
    {
        bringup.start(fake_now, 4, counts());
        for (std::uint64_t t = 0; t < duration_us && bringup.state() == BringupState::Running; t += cycle_us) {
            fake_now += cycle_us;
            const auto duties = bringup.step(fake_now, counts(), settings);
            for (std::size_t i = 0; i < 4; ++i) {
                motors[i]->drive(duties[i]);
            }
        }
    }

    [[nodiscard]] const OutputResult& result(std::size_t output) const { return bringup.result(output - 1); }

    std::array<std::unique_ptr<vk::sim::Motor>, 4> motors;
    std::array<std::size_t, 4> encoder_of{0, 1, 2, 3};  // which motor each output's encoder is on
    BringupCase bringup;
    Settings settings{};
};

TEST_F(BringupTest, EveryCorrectlyWiredOutputPasses)
{
    run();
    ASSERT_EQ(bringup.state(), BringupState::Done);
    for (std::size_t output = 1; output <= 4; ++output) {
        EXPECT_EQ(result(output).result, BringupResult::Pass) << "output " << output;
        EXPECT_EQ(result(output).encoder, output);
        EXPECT_EQ(result(output).sign, 1);
    }
}

TEST_F(BringupTest, AReversedMotorIsFoundAndMotRevFixesIt)
{
    motors[1]->set_reversed(true);
    run();
    EXPECT_EQ(result(2).result, BringupResult::Reversed);
    EXPECT_EQ(result(2).encoder, 2);
    EXPECT_EQ(result(2).sign, -1);
    EXPECT_EQ(result(1).result, BringupResult::Pass);

    settings.outputs[1].reversed = true;
    run();
    EXPECT_EQ(result(2).result, BringupResult::Pass);
}

TEST_F(BringupTest, AReversedEncoderIsFoundAndMotEncRevFixesIt)
{
    motors[2]->set_encoder_reversed(true);
    run();
    EXPECT_EQ(result(3).result, BringupResult::Reversed);
    settings.outputs[2].encoder_reversed = true;
    run();
    EXPECT_EQ(result(3).result, BringupResult::Pass);
}

TEST_F(BringupTest, ADeadMotorMovesNoEncoder)
{
    motors[0]->set_powered(false);
    run();
    EXPECT_EQ(result(1).result, BringupResult::NotMoved);
    EXPECT_EQ(result(1).encoder, 0);
    EXPECT_EQ(result(1).sign, 0);
}

TEST_F(BringupTest, SwappedEncodersNameTheEncoderThatMoved)
{
    encoder_of = {1, 0, 2, 3};
    run();
    EXPECT_EQ(result(1).result, BringupResult::OtherEncoder);
    EXPECT_EQ(result(1).encoder, 2);
    EXPECT_EQ(result(2).result, BringupResult::OtherEncoder);
    EXPECT_EQ(result(2).encoder, 1);
    EXPECT_EQ(result(3).result, BringupResult::Pass);
}

TEST_F(BringupTest, StaysWithinTheOutputLimit)
{
    settings.loop.out_max = 0.1F;
    bringup.start(fake_now, 4, counts());
    fake_now += cycle_us;
    EXPECT_FLOAT_EQ(bringup.step(fake_now, counts(), settings)[0], 0.1F);
}

TEST_F(BringupTest, AbortStopsEveryOutput)
{
    run(BringupCase::pulse_us / 2);
    bringup.abort();
    EXPECT_EQ(bringup.state(), BringupState::Aborted);
    fake_now += cycle_us;
    const auto duties = bringup.step(fake_now, counts(), settings);
    for (const float duty : duties) {
        EXPECT_FLOAT_EQ(duty, 0.0F);
    }
}

}  // namespace
