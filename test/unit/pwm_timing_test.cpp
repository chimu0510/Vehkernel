// SPDX-License-Identifier: GPL-3.0-or-later
// Unit tests for the STM32F4 motor PWM settings (spec 0006): the prescaler
// and period for each MOT_PWM_FREQ, and the advanced timer's output state
// when its main outputs are disabled.
#include <vk/platform/pwm_timing.hpp>

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using vk::platform::pwm_timing;

constexpr std::uint32_t c50c_apb2_timer_hz = 168'000'000;

std::uint32_t frequency(std::uint32_t timer_hz, const vk::platform::PwmTiming& timing)
{
    return timer_hz / ((timing.prescaler + 1) * (timing.period + 1));
}

TEST(PwmTiming, ReachesOneTenAndTwentyKilohertzWithAtLeastAThousandSteps)
{
    for (const std::uint32_t hz : {1'000U, 10'000U, 20'000U}) {
        const auto timing = pwm_timing(c50c_apb2_timer_hz, hz);
        ASSERT_TRUE(timing.has_value()) << hz;
        const vk::platform::PwmTiming settings = timing.value_or(vk::platform::PwmTiming{});
        EXPECT_GE(settings.period + 1, 1000U) << hz;
        EXPECT_LE(settings.period, 0xFFFFU) << hz;
        EXPECT_NEAR(static_cast<double>(frequency(c50c_apb2_timer_hz, settings)), hz, hz * 0.01) << hz;
    }
    // At 10 kHz no prescaler is needed: 16800 steps.
    EXPECT_EQ(pwm_timing(c50c_apb2_timer_hz, 10'000), (vk::platform::PwmTiming{.prescaler = 0, .period = 16'799}));
}

TEST(PwmTiming, AnUnreachableFrequencyHasNoSettings)
{
    EXPECT_FALSE(pwm_timing(c50c_apb2_timer_hz, 0).has_value());
    EXPECT_FALSE(pwm_timing(c50c_apb2_timer_hz, 200'000).has_value());  // 840 steps
}

TEST(PwmTiming, TheOutputsDriveLowWhenTheMainOutputsAreDisabled)
{
    EXPECT_NE(vk::platform::advanced_timer_bdtr & vk::platform::advanced_timer_ossi, 0U);
    EXPECT_NE(vk::platform::advanced_timer_bdtr & vk::platform::advanced_timer_moe, 0U);
    EXPECT_EQ(vk::platform::advanced_timer_ois & vk::platform::advanced_timer_ois_mask, 0U);
}

}  // namespace
