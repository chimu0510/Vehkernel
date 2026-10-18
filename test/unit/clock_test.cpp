// Microsecond clock (ADR-0008): the SITL clock, and the STM32F4 counter
// extension logic, which is free of hardware access.
#include <vk/hal/clock.hpp>
#include <vk/platform/counter_extension.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <type_traits>

namespace {

using vk::platform::extend_counter;

TEST(SystemClock, IsSixtyFourBitsWide)
{
    static_assert(std::is_same_v<decltype(vk::hal::SystemClock::now_us()), std::uint64_t>);
}

TEST(SystemClock, NeverGoesBackwards)
{
    std::uint64_t previous = vk::hal::SystemClock::now_us();
    for (int i = 0; i < 100000; ++i) {
        const std::uint64_t now = vk::hal::SystemClock::now_us();
        ASSERT_GE(now, previous);
        previous = now;
    }
}

TEST(SystemClock, AdvancesInMicroseconds)
{
    const std::uint64_t start = vk::hal::SystemClock::now_us();
    std::uint64_t now = start;
    while (now - start < 2000) {
        now = vk::hal::SystemClock::now_us();
    }
    EXPECT_LT(now - start, 1'000'000U);
}

TEST(CounterExtension, CombinesOverflowsAndCounter)
{
    EXPECT_EQ(extend_counter(0, 0, false), 0U);
    EXPECT_EQ(extend_counter(0, 1234, false), 1234U);
    EXPECT_EQ(extend_counter(3, 5, false), (3U << 16U) + 5U);
}

TEST(CounterExtension, CountsAPendingWrapWhenTheCounterHasWrapped)
{
    EXPECT_EQ(extend_counter(7, 2, true), (8U << 16U) + 2U);
}

TEST(CounterExtension, IgnoresAPendingWrapWhenTheCounterWasReadBeforeIt)
{
    EXPECT_EQ(extend_counter(7, 0xFFFF, true), (7U << 16U) + 0xFFFFU);
}

TEST(CounterExtension, StaysMonotonicAcrossAWrap)
{
    const std::uint64_t before = extend_counter(9, 0xFFFF, false);
    const std::uint64_t wrapped_not_counted = extend_counter(9, 0, true);
    const std::uint64_t wrapped_counted = extend_counter(10, 1, false);
    EXPECT_LT(before, wrapped_not_counted);
    EXPECT_LT(wrapped_not_counted, wrapped_counted);
}

TEST(CounterExtension, GoesBeyondThirtyTwoBits)
{
    // 2^32 us is about 71.6 minutes; a vehicle runs longer than that.
    const std::uint64_t overflows = (std::uint64_t{1} << 16U) + 1U;
    EXPECT_GT(extend_counter(overflows, 0, false), std::uint64_t{0xFFFFFFFF});
}

}  // namespace
