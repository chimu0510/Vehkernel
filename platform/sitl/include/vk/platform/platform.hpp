// Platform start-up for SITL (ADR-0009, ADR-0018).
#pragma once

#include <vk/core/work_queue.hpp>
#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/motion.hpp>

#include <cstdint>

namespace vk::platform {

// SITL's simulated board: an ICM20948 at 0x68 on a simulated I2C bus, read
// at 200 Hz like the C50C V1.1's timing preset.
inline constexpr std::uint8_t imu_address = 0x68;
inline constexpr std::uint32_t imu_rate_hz = 200;

// The simulated I2C bus the IMU is attached to.
sim::I2cBus& imu_bus();

// Starts the microsecond clock and attaches the simulated devices, the IMU
// following `motion`. Call once from the entry point, before anything else runs.
void init(sim::MotionProfile motion = sim::MotionProfile::Still);

// Publishes the simulated IMU's true attitude on vehicle_attitude_groundtruth
// for every imu_sample, from a work item on `queue`.
void start_ground_truth(core::WorkQueue& queue);

// SITL's board senses actuator power (ADR-0005). The simulated power is on
// unless cut, at start or later over the Link's sim_actuator_power frame.
bool actuator_power();
void set_actuator_power(bool powered);

// Stops the scheduler after the given run time, so a SITL run ends on its own.
void stop_after(std::uint64_t duration_us);

}  // namespace vk::platform
