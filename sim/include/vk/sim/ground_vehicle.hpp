// SPDX-License-Identifier: GPL-3.0-or-later
// SITL's ground vehicle (ADR-0018): a rigid body the simulated motors move,
// so the control chain and the odometry are checked against a truth.
//
// The wheels' true speeds give the body twist through the frame's wheel
// kinematics (the WHEELED vehicle model's J⁺, ADR-0013), reduced by the
// slip: with slip s the vehicle moves and turns (1 - s) as far as its
// wheels say, as when its tyres skid. The pose integrates the twist in the
// local level frame (north-east-down, origin and heading zero at boot) with
// the twist's average over each step. The vehicle stays level.
//
// The model advances to the clock's time whenever it is read, as the motors
// do; SITL reads it with every IMU sample, so its steps are the IMU's.
#pragma once

#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/sim/icm20948.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::sim {

class GroundVehicle {
public:
    using Clock = std::uint64_t (*)() noexcept;
    // The true speed [rad/s] of the wheel on output `index` (0 for output 1).
    using WheelSpeed = float (*)(std::size_t index);

    struct Config {
        estimation::WheelKinematics kinematics;
        std::array<std::size_t, estimation::max_wheels> outputs{};  // each wheel's output index
        float slip = 0.0F;  // 0 to 1: the fraction of the wheels' motion lost
    };

    struct State {
        std::uint64_t timestamp = 0;  // [us]
        double north = 0.0;           // [m]
        double east = 0.0;            // [m]
        float yaw = 0.0F;             // [rad] from north, positive turning right
        estimation::Twist twist{};    // in the body frame
        estimation::Vector3 acceleration;  // [m/s^2] linear, in the body frame

        [[nodiscard]] estimation::Quaternion attitude() const
        {
            return estimation::Quaternion::from_euler({.yaw = yaw});
        }
    };

    GroundVehicle(const Config& config, Clock clock, WheelSpeed wheel_speed);

    // The state now.
    const State& state();

private:
    [[nodiscard]] estimation::Twist wheel_twist() const;

    Config config_;
    Clock clock_;
    WheelSpeed wheel_speed_;
    State state_{};
};

// Drives a simulated ICM20948 from a ground vehicle: the gyro reads the
// vehicle's yaw rate plus a constant bias, the accelerometer the specific
// force of a level body plus its linear acceleration, both in the sensor
// frame (x forward, y left, z up).
class VehicleImu final : public Icm20948::OutputSource {
public:
    VehicleImu(GroundVehicle& vehicle, const estimation::Vector3& gyro_bias)
        : vehicle_(vehicle), gyro_bias_(gyro_bias)
    {
    }

    VehicleImu(const VehicleImu&) = delete;
    VehicleImu& operator=(const VehicleImu&) = delete;
    VehicleImu(VehicleImu&&) = delete;
    VehicleImu& operator=(VehicleImu&&) = delete;
    ~VehicleImu() = default;

    void update(Icm20948& imu) override;

private:
    GroundVehicle& vehicle_;
    estimation::Vector3 gyro_bias_;
};

}  // namespace vk::sim
