// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/sim/ground_vehicle.hpp>

#include <vk/estimation/models/accelerometer_model.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace vk::sim {

using estimation::Twist;
using estimation::Vector3;

GroundVehicle::GroundVehicle(const Config& config, Clock clock, WheelSpeed wheel_speed)
    : config_(config), clock_(clock), wheel_speed_(wheel_speed)
{
    config_.slip = std::clamp(config_.slip, 0.0F, 1.0F);
    state_.timestamp = clock_();
    state_.twist = wheel_twist();
}

Twist GroundVehicle::wheel_twist() const
{
    const auto& k = config_.kinematics;
    std::array<float, estimation::max_wheels> speeds{};
    for (std::size_t i = 0; i < k.size(); ++i) {
        speeds[i] = wheel_speed_(config_.outputs[i]);
    }
    const Twist t = k.measure(std::span(speeds).first(k.size())).twist;
    const float kept = 1.0F - config_.slip;
    return {.vx = t.vx * kept, .vy = t.vy * kept, .yaw_rate = t.yaw_rate * kept};
}

const GroundVehicle::State& GroundVehicle::state()
{
    const std::uint64_t now = clock_();
    if (now <= state_.timestamp) {
        return state_;
    }
    const float dt = static_cast<float>(now - state_.timestamp) * 1e-6F;
    const Twist start = state_.twist;
    const Twist end = wheel_twist();
    // The twist's average over the step, applied at the step's mid heading.
    const float vx = (start.vx + end.vx) / 2.0F;
    const float vy = (start.vy + end.vy) / 2.0F;
    const float yaw_rate = (start.yaw_rate + end.yaw_rate) / 2.0F;
    const float mid = state_.yaw + yaw_rate * dt / 2.0F;
    state_.north += static_cast<double>((vx * std::cos(mid) - vy * std::sin(mid)) * dt);
    state_.east += static_cast<double>((vx * std::sin(mid) + vy * std::cos(mid)) * dt);
    state_.yaw = std::remainder(state_.yaw + yaw_rate * dt, 2.0F * std::numbers::pi_v<float>);
    // Linear acceleration in the rotating body frame: dv/dt + ω × v.
    state_.acceleration = {(end.vx - start.vx) / dt - yaw_rate * vy, (end.vy - start.vy) / dt + yaw_rate * vx, 0.0F};
    state_.twist = end;
    state_.timestamp = now;
    return state_;
}

void VehicleImu::update(Icm20948& imu)
{
    const GroundVehicle::State& truth = vehicle_.state();
    const Vector3 gyro = Vector3{0.0F, 0.0F, truth.twist.yaw_rate} + gyro_bias_;
    const Vector3 accel = estimation::AccelerometerModel::predict(truth.attitude()) + truth.acceleration;
    // Body (x forward, y right, z down) to sensor (x forward, y left, z up).
    imu.set_gyro({gyro.x, -gyro.y, -gyro.z});
    imu.set_accel({accel.x, -accel.y, -accel.z});
}

}  // namespace vk::sim
