// SPDX-License-Identifier: GPL-3.0-or-later
// Unit tests for the vehicle rest detector (ADR-0014, spec 0012).
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/rest_detector.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

namespace {

using vk::estimation::GyroAverage;
using vk::estimation::RestDetector;
using vk::estimation::Vector3;

const Vector3 still_accel{0.0F, 0.0F, -9.80665F};
const Vector3 still_gyro{0.001F, -0.001F, 0.002F};
constexpr std::uint64_t dt_us = 5'000;  // 200 Hz IMU rate

TEST(RestDetector, NoBodyVelocityNeverAtRest)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;

    // Feed still IMU for 2 seconds with no velocity ever fed.
    for (int i = 0; i < 400; ++i) {
        const auto avg = detector.feed_imu(now_us, still_gyro, still_accel);
        EXPECT_FALSE(avg.has_value());
        now_us += dt_us;
    }

    EXPECT_FALSE(detector.is_still());
    EXPECT_FALSE(detector.at_rest());
}

TEST(RestDetector, EntersRestAfterContinuous05Seconds)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;

    // Feed still velocity (zero speed and yaw rate) at 50 Hz, and IMU at 200 Hz.
    // 0.49 s (98 samples): still, but not at rest yet (< 0.5 s).
    for (int i = 0; i < 98; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        const auto avg = detector.feed_imu(now_us, still_gyro, still_accel);
        EXPECT_FALSE(avg.has_value());
        now_us += dt_us;
    }

    EXPECT_TRUE(detector.is_still());
    EXPECT_FALSE(detector.at_rest());

    // 0.50 s (sample 100): reached 0.5 s continuous stillness -> enters rest.
    for (int i = 98; i < 101; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, still_gyro, still_accel);
        now_us += dt_us;
    }

    EXPECT_TRUE(detector.at_rest());
}

TEST(RestDetector, AveragesRawGyroOver02SecondsAndEmits)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;
    // A constant reading below the stillness threshold from the start, so the window holds only it.
    const Vector3 gyro_bias{0.02F, -0.015F, 0.03F};

    // 0.5 s to reach rest:
    for (int i = 0; i < 101; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, gyro_bias, still_accel);
        now_us += dt_us;
    }
    ASSERT_TRUE(detector.at_rest());

    // Now at rest: the window started on the last sample above; 40 more samples complete 0.2 s.
    std::optional<GyroAverage> finished_avg;

    for (int i = 0; i < 41; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        const auto avg = detector.feed_imu(now_us, gyro_bias, still_accel);
        if (avg.has_value()) {
            finished_avg = avg;
        }
        now_us += dt_us;
    }

    ASSERT_TRUE(finished_avg.has_value());
    const auto got = finished_avg.value_or(vk::estimation::GyroAverage{});
    // 1e-4F absorbs float rounding in summing and dividing 41 equal readings.
    EXPECT_NEAR(got.mean_gyro.x, gyro_bias.x, 1e-4F);
    EXPECT_NEAR(got.mean_gyro.y, gyro_bias.y, 1e-4F);
    EXPECT_NEAR(got.mean_gyro.z, gyro_bias.z, 1e-4F);
    // 0.01F absorbs discrete sample step (5 ms) duration resolution.
    EXPECT_NEAR(got.duration_seconds, 0.2F, 0.01F);
}

TEST(RestDetector, MovingImuLeavesRestAndDropsPartialWindow)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;

    // Reach rest (0.5 s):
    for (int i = 0; i < 101; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, still_gyro, still_accel);
        now_us += dt_us;
    }
    ASSERT_TRUE(detector.at_rest());

    // Accumulate 20 samples in window (0.1 s):
    for (int i = 0; i < 20; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, still_gyro, still_accel);
        now_us += dt_us;
    }
    // The sample that reached rest opened the window, then 20 more.
    EXPECT_EQ(detector.window_samples(), 21U);

    // IMU motion (gyro rate exceeds still_gyro threshold):
    const Vector3 moving_gyro{0.2F, 0.0F, 0.0F};
    detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
    const auto avg = detector.feed_imu(now_us, moving_gyro, still_accel);
    EXPECT_FALSE(avg.has_value());
    EXPECT_FALSE(detector.is_still());
    EXPECT_FALSE(detector.at_rest());
    EXPECT_EQ(detector.window_samples(), 0U);
}

TEST(RestDetector, MovingBodyVelocityLeavesRest)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;

    // Reach rest (0.5 s):
    for (int i = 0; i < 101; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, still_gyro, still_accel);
        now_us += dt_us;
    }
    ASSERT_TRUE(detector.at_rest());

    // Vehicle starts moving straight: vx = 0.5 m/s.
    detector.feed_velocity(now_us, 0.5F, 0.0F, 0.0F);
    const auto avg = detector.feed_imu(now_us, still_gyro, still_accel);
    EXPECT_FALSE(avg.has_value());
    EXPECT_FALSE(detector.at_rest());
    EXPECT_FALSE(detector.is_still());
}

TEST(RestDetector, StaleBodyVelocityLeavesRest)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;

    // Reach rest (0.5 s):
    for (int i = 0; i < 101; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, still_gyro, still_accel);
        now_us += dt_us;
    }
    ASSERT_TRUE(detector.at_rest());

    // Stop sending velocity updates. After 60 ms (> velocity_max_age_us 50 ms):
    now_us += 60'000;
    const auto avg = detector.feed_imu(now_us, still_gyro, still_accel);
    EXPECT_FALSE(avg.has_value());
    EXPECT_FALSE(detector.at_rest());
}

TEST(RestDetector, ResetClearsState)
{
    RestDetector detector;
    std::uint64_t now_us = 1'000'000;

    for (int i = 0; i < 101; ++i) {
        if (i % 4 == 0) {
            detector.feed_velocity(now_us, 0.0F, 0.0F, 0.0F);
        }
        detector.feed_imu(now_us, still_gyro, still_accel);
        now_us += dt_us;
    }
    ASSERT_TRUE(detector.at_rest());

    detector.reset();
    EXPECT_FALSE(detector.is_still());
    EXPECT_FALSE(detector.at_rest());
    EXPECT_EQ(detector.window_samples(), 0U);
}

}  // namespace
