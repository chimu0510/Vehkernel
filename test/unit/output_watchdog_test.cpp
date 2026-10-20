// Unit tests for the output watchdog (ADR-0005, spec 0006).
#include <vk/hal/output_watchdog.hpp>

#include <gtest/gtest.h>

namespace {

constexpr std::uint32_t period_us = 5'000;

TEST(OutputWatchdog, TripsAfterThreePeriodsWithoutARefreshAndNotBefore)
{
    vk::hal::OutputWatchdog watchdog(period_us);
    EXPECT_FALSE(watchdog.expired(1'000'000));  // never refreshed: nothing to stop yet
    watchdog.refresh(1'000'000);
    EXPECT_FALSE(watchdog.expired(1'000'000 + 3 * period_us));
    EXPECT_TRUE(watchdog.expired(1'000'000 + 3 * period_us + 1));
    EXPECT_EQ(watchdog.trips(), 1U);
}

TEST(OutputWatchdog, CountsEachStallOnce)
{
    vk::hal::OutputWatchdog watchdog(period_us);
    watchdog.refresh(1'000'000);
    EXPECT_TRUE(watchdog.expired(1'100'000));
    EXPECT_TRUE(watchdog.expired(1'200'000));
    EXPECT_EQ(watchdog.trips(), 1U);
    watchdog.refresh(1'300'000);
    EXPECT_FALSE(watchdog.expired(1'305'000));
    EXPECT_TRUE(watchdog.expired(1'400'000));
    EXPECT_EQ(watchdog.trips(), 2U);
}

}  // namespace
