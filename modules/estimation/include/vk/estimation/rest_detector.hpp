// SPDX-License-Identifier: GPL-3.0-or-later
// Vehicle rest detector for zero rate gyro updates (ADR-0014, spec 0012).
//
// Pure logic, independent of the bus: determines when the vehicle is at rest
// by combining IMU stillness (raw gyro and gravity norm) and body velocity
// measurements. While at rest, accumulates the raw gyro and hands out finished
// averages over window_us. Leaving rest drops the partial window.
#pragma once

#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/still_alignment.hpp>

#include <cmath>
#include <cstdint>
#include <optional>

namespace vk::estimation {

struct RestDetectorConfig {
    StillAlignmentConfig imu{};
    std::uint64_t rest_us = 500'000;            // [us] continuous stillness required before entering rest (0.5 s)
    std::uint64_t velocity_max_age_us = 50'000;  // [us] maximum age of body velocity measurement (50 ms)
    float still_speed = 0.01F;                  // [m/s] |vx|, |vy| below this counted as still
    float still_yaw_rate = 0.01F;               // [rad/s] |yaw_rate| below this counted as still
    std::uint64_t window_us = 200'000;          // [us] gyro averaging window duration (0.2 s)
};

struct GyroAverage {
    Vector3 mean_gyro{};
    float duration_seconds = 0.0F;
};

class RestDetector {
public:
    using Config = RestDetectorConfig;

    explicit RestDetector(const RestDetectorConfig& config = {}) : config_(config) {}

    // Resets rest detector state.
    void reset()
    {
        still_ = false;
        still_since_us_ = 0;
        last_imu_ts_us_ = 0;
        window_samples_ = 0;
        window_start_us_ = 0;
        window_sum_ = {};
        have_velocity_ = false;
        velocity_ts_us_ = 0;
        vx_ = 0.0F;
        vy_ = 0.0F;
        yaw_rate_ = 0.0F;
    }

    // Feeds a body velocity measurement.
    void feed_velocity(std::uint64_t timestamp_us, float vx, float vy, float yaw_rate)
    {
        have_velocity_ = true;
        velocity_ts_us_ = timestamp_us;
        vx_ = vx;
        vy_ = vy;
        yaw_rate_ = yaw_rate;
    }

    // Feeds an IMU sample. Returns a finished gyro average if one just completed.
    std::optional<GyroAverage> feed_imu(std::uint64_t timestamp_us, const Vector3& gyro, const Vector3& accel)
    {
        last_imu_ts_us_ = timestamp_us;

        // 1. Check IMU stillness (same thresholds as StillAlignment)
        const float g = config_.imu.gravity;
        const bool imu_still = gyro.norm() < config_.imu.still_gyro &&
                               std::abs(accel.norm() - g) < config_.imu.still_accel * g;

        // 2. Check body velocity stillness and freshness
        bool vel_ok = false;
        if (have_velocity_) {
            const bool fresh = (timestamp_us >= velocity_ts_us_)
                                   ? (timestamp_us - velocity_ts_us_ <= config_.velocity_max_age_us)
                                   : (velocity_ts_us_ - timestamp_us <= config_.velocity_max_age_us);
            const bool slow = std::abs(vx_) < config_.still_speed &&
                              std::abs(vy_) < config_.still_speed &&
                              std::abs(yaw_rate_) < config_.still_yaw_rate;
            vel_ok = fresh && slow;
        }

        const bool is_currently_still = imu_still && vel_ok;

        if (!is_currently_still) {
            // Leaving rest drops the partial window and resets stillness tracking
            still_ = false;
            still_since_us_ = 0;
            window_samples_ = 0;
            window_start_us_ = 0;
            window_sum_ = {};
            return std::nullopt;
        }

        if (!still_) {
            still_ = true;
            still_since_us_ = timestamp_us;
        }

        // Must be continuously still for at least rest_us to be at rest
        if ((timestamp_us - still_since_us_) < config_.rest_us) {
            return std::nullopt;
        }

        // At rest: average raw gyro over window_us
        if (window_samples_ == 0) {
            window_start_us_ = timestamp_us;
        }
        window_sum_ += gyro;
        ++window_samples_;

        if ((timestamp_us - window_start_us_) >= config_.window_us) {
            const float duration_s = static_cast<float>(timestamp_us - window_start_us_) * 1e-6F;
            const Vector3 mean = window_sum_ * (1.0F / static_cast<float>(window_samples_));
            // Reset window for the next period
            window_samples_ = 0;
            window_start_us_ = 0;
            window_sum_ = {};
            return GyroAverage{.mean_gyro = mean,
                               .duration_seconds = duration_s > 0.0F
                                                       ? duration_s
                                                       : static_cast<float>(config_.window_us) * 1e-6F};
        }

        return std::nullopt;
    }

    [[nodiscard]] bool is_still() const { return still_; }
    [[nodiscard]] bool at_rest() const
    {
        return still_ && ((last_imu_ts_us_ - still_since_us_) >= config_.rest_us);
    }
    [[nodiscard]] std::uint32_t window_samples() const { return window_samples_; }
    [[nodiscard]] const Config& config() const { return config_; }

private:
    Config config_;
    bool still_ = false;
    std::uint64_t still_since_us_ = 0;
    std::uint64_t last_imu_ts_us_ = 0;

    std::uint32_t window_samples_ = 0;
    std::uint64_t window_start_us_ = 0;
    Vector3 window_sum_{};

    bool have_velocity_ = false;
    std::uint64_t velocity_ts_us_ = 0;
    float vx_ = 0.0F;
    float vy_ = 0.0F;
    float yaw_rate_ = 0.0F;
};

}  // namespace vk::estimation
