// SPDX-License-Identifier: GPL-3.0-or-later
// Still-vehicle alignment helper for attitude estimators (ADR-0014, spec 0012).
//
// Shared by the Mahony and EKF estimators so both start alike (spec 0012).
// Accumulates the mean gyro reading, mean specific force, and (when given) mean
// magnetometer reading while the vehicle remains still for alignment_us. Movement
// restarts the wait.
#pragma once

#include <vk/estimation/math/vector3.hpp>

#include <cmath>
#include <cstdint>

namespace vk::estimation {

struct StillAlignmentConfig {
    std::uint64_t alignment_us = 1'000'000;  // stillness needed to align
    float still_gyro = 0.05F;                // [rad/s] largest gyro norm counted as still
    float still_accel = 0.05F;               // largest | |a| - g | counted as still, as a fraction of g
    float gravity = 9.80665F;                // [m/s^2]
};

class StillAlignment {
public:
    using Config = StillAlignmentConfig;

    explicit StillAlignment(const StillAlignmentConfig& config) : config_(config) {}

    // Resets alignment state.
    void reset()
    {
        still_since_us_ = 0;
        still_samples_ = 0;
        gyro_sum_ = {};
        accel_sum_ = {};
        mag_sum_ = {};
        mag_samples_ = 0;
    }

    // Feeds an IMU sample. Returns true when alignment has just completed.
    bool feed_imu(std::uint64_t timestamp_us, const Vector3& gyro, const Vector3& accel)
    {
        const float g = config_.gravity;
        const bool still = gyro.norm() < config_.still_gyro &&
                           std::abs(accel.norm() - g) < config_.still_accel * g;
        if (!still) {
            reset();
            return false;
        }
        if (still_samples_ == 0) {
            still_since_us_ = timestamp_us;
            gyro_sum_ = {};
            accel_sum_ = {};
            mag_sum_ = {};
            mag_samples_ = 0;
        }
        ++still_samples_;
        gyro_sum_ += gyro;
        accel_sum_ += accel;
        return (timestamp_us - still_since_us_) >= config_.alignment_us;
    }

    // Feeds a magnetometer sample while aligning. Only counts if vehicle is currently still.
    void feed_mag(const Vector3& field)
    {
        if (still_samples_ > 0) {
            mag_sum_ += field;
            ++mag_samples_;
        }
    }

    [[nodiscard]] bool is_still() const { return still_samples_ > 0; }
    [[nodiscard]] std::uint32_t still_samples() const { return still_samples_; }
    [[nodiscard]] std::uint32_t mag_samples() const { return mag_samples_; }
    [[nodiscard]] bool has_mag() const { return mag_samples_ > 0; }

    [[nodiscard]] Vector3 mean_accel() const
    {
        if (still_samples_ == 0) {
            return {};
        }
        return accel_sum_ * (1.0F / static_cast<float>(still_samples_));
    }

    [[nodiscard]] Vector3 mean_gyro() const
    {
        if (still_samples_ == 0) {
            return {};
        }
        return gyro_sum_ * (1.0F / static_cast<float>(still_samples_));
    }

    [[nodiscard]] Vector3 mean_mag() const
    {
        if (mag_samples_ == 0) {
            return {};
        }
        return mag_sum_ * (1.0F / static_cast<float>(mag_samples_));
    }

    [[nodiscard]] float mean_mag_strength() const
    {
        return mean_mag().norm();
    }

private:
    StillAlignmentConfig config_;
    std::uint64_t still_since_us_ = 0;
    std::uint32_t still_samples_ = 0;
    std::uint32_t mag_samples_ = 0;
    Vector3 gyro_sum_{};
    Vector3 accel_sum_{};
    Vector3 mag_sum_{};
};

}  // namespace vk::estimation
