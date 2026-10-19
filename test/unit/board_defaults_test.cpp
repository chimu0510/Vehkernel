// Board defaults as compile-time constants (ADR-0012, ADR-0015), checked
// against the C50C V1.1 board header the board generator emits (its golden copy).
#include <board.hpp>

#include <gtest/gtest.h>

namespace {

namespace board = vk::board;

static_assert(board::timing::trigger == vk::hal::TimingTrigger::Timer);
static_assert(board::timing::base_rate_hz == 200);
static_assert(board::timing::sampling == vk::hal::ImuSampling::PerSample);
static_assert(board::devices::IMU0::rotation == vk::hal::Rotation::Yaw90);  // measured in #35

TEST(BoardDefaults, ExposeTheTimingPreset)
{
    EXPECT_EQ(board::defaults::CTRL_RATE_HZ, 200);
    EXPECT_EQ(board::defaults::CTRL_TRIGGER, "TIMER");
    EXPECT_EQ(board::defaults::IMU_SAMPLING, "PER_SAMPLE");
}

TEST(BoardDefaults, ExposeOtherBoardDefaults)
{
    EXPECT_EQ(board::defaults::SERIAL4_FUNCTION, "RC_INPUT");
}

}  // namespace
