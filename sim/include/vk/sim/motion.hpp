// SPDX-License-Identifier: GPL-3.0-or-later
// Scripted motion of SITL's simulated IMU, with its true attitude (ADR-0018).
//
// A profile is a few segments of constant body rate, plus linear
// acceleration bursts, starting once the estimator has had time to align. The
// truth at any instant follows in closed form, so the simulated IMU and the
// ground truth topic agree exactly. Frames: body x forward, y right, z down;
// level frame north-east-down.
#pragma once

#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/sim/icm20948.hpp>

#include <cstdint>
#include <optional>
#include <string_view>

namespace vk::sim {

enum class MotionProfile : std::uint8_t {
    Still,   // level, at rest
    Tilt,    // 20 deg of roll, then 20 deg of pitch
    Rotate,  // 90 deg turns about x, y and z in turn, at 45 deg/s
    Shake,   // level, with 6 m/s^2 bursts forward
};

std::optional<MotionProfile> motion_profile(std::string_view name);

struct MotionState {
    estimation::Quaternion attitude;         // body to level frame
    estimation::Vector3 rate;                // body rate [rad/s]
    estimation::Vector3 linear_acceleration; // in the body frame [m/s^2]
};

// The truth `t_us` microseconds after boot.
MotionState motion_at(MotionProfile profile, std::uint64_t t_us);

// Drives a simulated ICM20948 from a profile: the gyro reads the true rate
// plus a constant bias, the accelerometer the specific force of the true
// attitude plus the linear acceleration, both in the sensor frame (x forward,
// y left, z up).
class MotionImu final : public Icm20948::OutputSource {
public:
    using Clock = std::uint64_t (*)() noexcept;

    MotionImu(MotionProfile profile, const estimation::Vector3& gyro_bias, Clock clock)
        : profile_(profile), gyro_bias_(gyro_bias), clock_(clock)
    {
    }

    MotionImu(const MotionImu&) = delete;
    MotionImu& operator=(const MotionImu&) = delete;
    MotionImu(MotionImu&&) = delete;
    MotionImu& operator=(MotionImu&&) = delete;
    ~MotionImu() = default;

    void update(Icm20948& imu) override;

    [[nodiscard]] MotionProfile profile() const { return profile_; }

private:
    MotionProfile profile_;
    estimation::Vector3 gyro_bias_;
    Clock clock_;
};

}  // namespace vk::sim
