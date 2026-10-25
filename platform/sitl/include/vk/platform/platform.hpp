// SPDX-License-Identifier: GPL-3.0-or-later
// Platform start-up for SITL (ADR-0009, ADR-0018).
#pragma once

#include <vk/core/fault.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/actuator.hpp>
#include <vk/hal/reset.hpp>
#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/sim/motor.hpp>
#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/motion.hpp>
#include <vk/sim/ssd1306.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::platform {

// SITL's simulated board: an ICM20948 at 0x68 on a simulated I2C bus, read
// at 200 Hz like the C50C V1.1's timing preset.
inline constexpr std::uint8_t imu_address = 0x68;
inline constexpr std::uint32_t imu_rate_hz = 200;

// The simulated I2C bus the IMU is attached to.
sim::I2cBus& imu_bus();

// SITL's display (ADR-0022): a simulated SSD1306, which is its own display bus.
sim::Ssd1306& display();

// SITL's user key (ADR-0022): released at start, pressed and released by the
// Link's sim_user_key frame.
bool user_key_pressed();
void set_user_key(bool pressed);

// Starts the microsecond clock and attaches the simulated devices, the IMU
// following `motion`. Call once from the entry point, before anything else runs.
void init(sim::MotionProfile motion = sim::MotionProfile::Still);

// Simulates a ground vehicle (ADR-0018) whose wheels, by `kinematics`, are
// the motors on `outputs` (indexes, 0 for output 1), losing the fraction
// `slip` of their motion. From then on it drives the IMU instead of the
// scripted motion. Call after init(), before the IMU starts.
void start_vehicle(const estimation::WheelKinematics& kinematics,
                   const std::array<std::size_t, estimation::max_wheels>& outputs, float slip);

// Publishes the simulated IMU's true attitude on vehicle_attitude_groundtruth
// for every imu_sample, from a work item on `queue`; with a ground vehicle,
// its attitude, and its pose and velocity on
// vehicle_local_odometry_groundtruth.
void start_ground_truth(core::WorkQueue& queue);

// SITL's board senses actuator power (ADR-0005). The simulated power is on
// unless cut, at start or later over the Link's sim_actuator_power frame;
// without it the simulated motors do not move.
bool actuator_power();
void set_actuator_power(bool powered);

// SITL's four actuator outputs, each a simulated motor with its encoder
// (spec 0006), available after init().
inline constexpr std::size_t actuator_count = 4;
std::span<const hal::MotorChannel> motor_channels();

// The simulated motor of an output, 1 to actuator_count.
sim::Motor& motor(std::size_t output);

// Starts the output watchdog (ADR-0005) for actuator control running every
// `period_us`, as a task above the work queues on the simulated clock.
void start_output_watchdog(std::uint32_t period_us);
std::uint32_t output_watchdog_trips();

// What caused the last reset (ADR-0005). SITL has no reset to read: each run
// starts as if powered on, unless whoever restarts it after a fault or a
// reboot says otherwise (--reset-cause).
core::ResetReport reset_report();
void set_reset_report(const core::ResetReport& report);

// The fault stop (ADR-0005) in SITL turns every simulated output off, prints
// "fault kind=<fault> address=<hex> duty=<d1>,<d2>,<d3>,<d4>" with the duty
// each motor is left with, and ends the run at once with this exit status.
inline constexpr int fault_exit_status = 12;

// A fault for tests (--fault): once any output has been driven with a duty
// other than zero for fault_after_driven_us, the output watchdog's task fails
// a configASSERT (ASSERT) or calls fail() (FAIL). Other faults have no SITL
// equivalent.
inline constexpr std::uint64_t fault_after_driven_us = 500'000;
void raise_fault_when_driven(core::Fault fault);

// Stops the scheduler after the given run time, so a SITL run ends on its own.
void stop_after(std::uint64_t duration_us);

// A reset ends SITL at once with an exit status that tells the targets apart
// (ADR-0023): whoever started it decides whether to start it again, or to
// run the SITL bootloader. It prints "reset target=firmware|bootloader" first.
inline constexpr int reset_firmware_exit_status = 10;
inline constexpr int reset_bootloader_exit_status = 11;
[[noreturn]] void reset(hal::BootTarget target);

}  // namespace vk::platform
