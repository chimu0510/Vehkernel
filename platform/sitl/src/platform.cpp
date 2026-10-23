// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/platform/platform.hpp>

#include <vk/core/scheduler.hpp>
#include <vk/hal/clock.hpp>

#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/icm20948.hpp>
#include <vk/sim/motion.hpp>
#include <vk/sim/ssd1306.hpp>

#include <vk/core/bus.hpp>
#include <vk/hal/output_watchdog.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace vk::platform {
namespace {

StaticTask_t stop_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> stop_stack;
std::uint64_t stop_duration_us = 0;

std::atomic<bool> simulated_actuator_power{true};
std::atomic<bool> simulated_user_key{false};

core::StaticInstance<hal::OutputWatchdog> watchdog;
StaticTask_t watchdog_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> watchdog_stack;

// One actuator output over a simulated motor: the duty and direction lines as
// an H-bridge applies them, and the encoder counter. Every duty refreshes the
// output watchdog.
class MotorOutput final : public hal::PwmOutput, public hal::DirectionLines, public hal::QuadratureEncoder {
public:
    void attach(sim::Motor& motor) { motor_ = &motor; }

    bool set_frequency(std::uint32_t /*hz*/) override { return true; }  // the model has no PWM

    void set_duty(float duty) override
    {
        duty_ = duty;
        if (watchdog.has_value()) {
            watchdog->refresh(hal::SystemClock::now_us());
        }
        apply();
    }

    // The watchdog's stop: the motor coasts until the next duty.
    void stop() { motor_->drive(0.0F); }

    void set(bool in1, bool in2) override
    {
        in1_ = in1;
        in2_ = in2;
        apply();
    }

    [[nodiscard]] std::uint16_t count() const override { return motor_->count(); }

private:
    void apply()
    {
        const float sign = in1_ && !in2_ ? 1.0F : (in2_ && !in1_ ? -1.0F : 0.0F);
        motor_->drive(sign * duty_);
    }

    sim::Motor* motor_ = nullptr;
    float duty_ = 0.0F;
    bool in1_ = false;
    bool in2_ = false;
};

std::array<core::StaticInstance<sim::Motor>, actuator_count> motors;
std::array<MotorOutput, actuator_count> motor_outputs;
std::array<hal::MotorChannel, actuator_count> channels{};

sim::I2cBus bus;
sim::Icm20948 imu;
sim::Ssd1306 oled;
// A small gyroscope bias in the body frame.
core::StaticInstance<sim::MotionImu> motion;

// Publishes the truth of each IMU sample at the sample's own time.
class GroundTruth final : public core::WorkItem {
public:
    explicit GroundTruth(core::WorkQueue& queue) : WorkItem("ground_truth", queue) {}

    void start() { trigger_on(core::topic<msg::ImuSample>(0)); }

protected:
    void run() override
    {
        msg::ImuSample sample{};
        while (imu_.copy(sample)) {
            const sim::MotionState truth = sim::motion_at(motion->profile(), sample.timestamp);
            msg::VehicleAttitudeGroundtruth out{};
            out.timestamp = sample.timestamp;
            out.q = truth.attitude.array();
            out.angular_velocity = truth.rate.array();
            publication_.publish(out);
        }
    }

private:
    core::Subscription<msg::ImuSample> imu_{0};
    core::Publication<msg::VehicleAttitudeGroundtruth> publication_;
};

core::StaticInstance<GroundTruth> ground_truth;

void stop_task(void* /*unused*/)
{
    // Highest priority, so the run ends on time whatever else is busy. Ticks
    // run slow on the POSIX port, so the end follows the microsecond clock.
    constexpr TickType_t check_ticks = 10;
    while (hal::SystemClock::now_us() < stop_duration_us) {
        vTaskDelay(check_ticks);
    }
    core::stop_scheduler();
}

}  // namespace

void init(sim::MotionProfile profile)
{
    // The first reading fixes the clock's zero at boot.
    static_cast<void>(hal::SystemClock::now_us());

    for (std::size_t i = 0; i < actuator_count; ++i) {
        motors[i].emplace(sim::Motor::Config{}, &hal::SystemClock::now_us);
        motor_outputs[i].attach(*motors[i]);
        channels[i] = {.pwm = &motor_outputs[i], .direction = &motor_outputs[i], .encoder = &motor_outputs[i]};
    }

    // The scripted motion, with sensor noise and a small gyroscope bias.
    motion.emplace(profile, estimation::Vector3{0.002F, 0.001F, -0.0005F}, &hal::SystemClock::now_us);
    imu.set_output_source(&*motion);
    imu.set_temperature(30.0F);
    imu.set_noise(0.05F, 0.005F);
    bus.attach(imu_address, imu);
}

sim::I2cBus& imu_bus()
{
    return bus;
}

sim::Ssd1306& display()
{
    return oled;
}

bool user_key_pressed()
{
    return simulated_user_key.load();
}

void set_user_key(bool pressed)
{
    simulated_user_key.store(pressed);
}

void start_ground_truth(core::WorkQueue& queue)
{
    ground_truth.emplace(queue);
    ground_truth->start();
}

void stop_after(std::uint64_t duration_us)
{
    stop_duration_us = duration_us;
    xTaskCreateStatic(stop_task, "sitl_stop", stop_stack.size(), nullptr, configMAX_PRIORITIES - 1,
                      stop_stack.data(), &stop_tcb);
}

void reset(hal::BootTarget target)
{
    const bool bootloader = target == hal::BootTarget::Bootloader;
    std::printf("reset target=%s\n", bootloader ? "bootloader" : "firmware");
    std::fflush(stdout);
    // At once, from whichever task asked: FreeRTOS threads must not run exit handlers.
    std::_Exit(bootloader ? reset_bootloader_exit_status : reset_firmware_exit_status);
}

bool actuator_power()
{
    return simulated_actuator_power.load();
}

void set_actuator_power(bool powered)
{
    simulated_actuator_power.store(powered);
    for (auto& m : motors) {
        if (m.has_value()) {
            m->set_powered(powered);
        }
    }
}

void start_output_watchdog(std::uint32_t period_us)
{
    watchdog.emplace(period_us);
    xTaskCreateStatic(
        [](void* /*unused*/) {
            // Above the work queues, so it runs when they stall.
            for (;;) {
                if (watchdog->expired(hal::SystemClock::now_us())) {
                    for (auto& output : motor_outputs) {
                        output.stop();
                    }
                }
                vTaskDelay(1);
            }
        },
        "watchdog", watchdog_stack.size(), nullptr, configMAX_PRIORITIES - 1, watchdog_stack.data(), &watchdog_tcb);
}

std::uint32_t output_watchdog_trips()
{
    return watchdog.has_value() ? watchdog->trips() : 0;
}

std::span<const hal::MotorChannel> motor_channels()
{
    return channels;
}

sim::Motor& motor(std::size_t output)
{
    return *motors.at(output - 1);
}

}  // namespace vk::platform
