// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/platform/platform.hpp>

#include <vk/core/fault.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/hal/clock.hpp>

#include <vk/sim/ground_vehicle.hpp>
#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/icm20948.hpp>
#include <vk/sim/motion.hpp>
#include <vk/sim/ssd1306.hpp>

#include <vk/core/bus.hpp>
#include <vk/hal/output_watchdog.hpp>
#include <vk/core/critical_section.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/msg/enums.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/manual_control_input.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>
#include <vk/msg/vehicle_local_odometry_groundtruth.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace vk::platform {
namespace {

StaticTask_t stop_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> stop_stack;
std::uint64_t stop_duration_us = 0;

std::atomic<bool> simulated_actuator_power{true};
std::atomic<bool> simulated_user_key{false};
std::atomic<std::uint64_t> user_key_release_us{0};  // 0: held until released

core::StaticInstance<hal::OutputWatchdog> watchdog;
StaticTask_t watchdog_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> watchdog_stack;

core::ResetReport last_reset{.cause = core::ResetCause::PowerOn};
core::Fault test_fault = core::Fault::None;  // --fault
std::uint64_t driven_since_us = 0;           // 0 while no output is driven

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
    void stop()
    {
        if (motor_ != nullptr) {
            motor_->drive(0.0F);
        }
    }

    [[nodiscard]] float duty() const { return motor_ == nullptr ? 0.0F : motor_->duty(); }

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
sim::Icm20948 imu(&hal::SystemClock::now_us);
sim::Ssd1306 oled;
sim::MagConfig mag_config{};
// A small gyroscope bias in the body frame.
constexpr estimation::Vector3 gyro_bias{0.002F, 0.001F, -0.0005F};
core::StaticInstance<sim::MotionImu> motion;
// The ground vehicle, once started; it then drives the IMU instead of motion.
core::StaticInstance<sim::GroundVehicle> vehicle;
core::StaticInstance<sim::VehicleImu> vehicle_imu;

float wheel_speed(std::size_t index)
{
    return motors.at(index)->speed();
}

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
            if (vehicle.has_value()) {
                publish_vehicle(sample.timestamp);
                continue;
            }
            const sim::MotionState truth = sim::motion_at(motion->profile(), sample.timestamp);
            msg::VehicleAttitudeGroundtruth out{};
            out.timestamp = sample.timestamp;
            out.q = truth.attitude.array();
            out.angular_velocity = truth.rate.array();
            publication_.publish(out);
        }
    }

private:
    // The vehicle's state now, a moment after the sample, stamped with it.
    void publish_vehicle(std::uint64_t timestamp)
    {
        const sim::GroundVehicle::State& truth = vehicle->state();
        const auto q = truth.attitude().array();
        const estimation::Vector3 rate{0.0F, 0.0F, truth.twist.yaw_rate};
        msg::VehicleAttitudeGroundtruth attitude{};
        attitude.timestamp = timestamp;
        attitude.q = q;
        attitude.angular_velocity = rate.array();
        publication_.publish(attitude);
        msg::VehicleLocalOdometryGroundtruth odometry{};
        odometry.timestamp = timestamp;
        odometry.position = {static_cast<float>(truth.north), static_cast<float>(truth.east), 0.0F};
        odometry.q = q;
        odometry.velocity = {truth.twist.vx, truth.twist.vy, 0.0F};
        odometry.angular_velocity = rate.array();
        odometry_publication_.publish(odometry);
    }

    core::Subscription<msg::ImuSample> imu_{0};
    core::Publication<msg::VehicleAttitudeGroundtruth> publication_;
    core::Publication<msg::VehicleLocalOdometryGroundtruth> odometry_publication_;
};

core::StaticInstance<GroundTruth> ground_truth;

// The simulated gamepad, set from the Link's work queue.
struct Gamepad {
    bool connected = false;
    bool silent = false;
    std::array<float, 4> sticks{};
    std::array<float, 2> triggers{};
    std::uint16_t buttons = 0;
};
Gamepad simulated_gamepad;  // guarded by the critical section

// Publishes the simulated gamepad as a gamepad driver would.
class SimGamepad final : public core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 20'000;

    explicit SimGamepad(core::WorkQueue& queue) : WorkItem("sim_gamepad", queue) {}

    void start() { schedule_periodic(interval_us); }

protected:
    void run() override
    {
        Gamepad now{};
        {
            const core::CriticalSection lock;
            now = simulated_gamepad;
        }
        if (now.silent || (!now.connected && !was_connected_)) {
            return;
        }
        was_connected_ = now.connected;
        msg::ManualControlInput input{};
        input.timestamp = queue().now_us();
        input.source = static_cast<std::uint8_t>(msg::ManualControlSource::Simulated);
        input.connected = now.connected;
        if (now.connected) {
            input.sticks = now.sticks;
            input.triggers = now.triggers;
            input.buttons = now.buttons;
        }
        publication_.publish(input);
    }

private:
    core::Publication<msg::ManualControlInput> publication_;
    bool was_connected_ = false;
};

core::StaticInstance<SimGamepad> sim_gamepad;

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

// Raises the --fault once an output has been driven for fault_after_driven_us.
void check_test_fault(std::uint64_t now_us)
{
    if (test_fault == core::Fault::None) {
        return;
    }
    const bool driven = std::ranges::any_of(motor_outputs, [](const MotorOutput& output) { return output.duty() != 0.0F; });
    if (!driven) {
        driven_since_us = 0;
        return;
    }
    if (driven_since_us == 0) {
        driven_since_us = now_us;
    }
    if (now_us - driven_since_us < fault_after_driven_us) {
        return;
    }
    if (test_fault == core::Fault::Assert) {
        configASSERT(false);
    }
    core::fail("SITL --fault");
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
    motion.emplace(profile, gyro_bias, &hal::SystemClock::now_us, mag_config);
    imu.set_output_source(&*motion);
    imu.set_temperature(30.0F);
    imu.set_noise(0.05F, 0.005F);
    imu.set_field_noise(0.3F);
    bus.attach(imu_address, imu);
}

sim::I2cBus& imu_bus()
{
    return bus;
}

sim::Icm20948& simulated_imu()
{
    return imu;
}

void set_mag_config(const sim::MagConfig& config)
{
    mag_config = config;
    if (motion.has_value()) {
        motion->set_mag_config(config);
    }
    if (vehicle_imu.has_value()) {
        vehicle_imu->set_mag_config(config);
    }
}

sim::Ssd1306& display()
{
    return oled;
}

bool user_key_pressed()
{
    const std::uint64_t release_us = user_key_release_us.load();
    return simulated_user_key.load() && (release_us == 0 || hal::SystemClock::now_us() < release_us);
}

void set_user_key(bool pressed, std::uint32_t hold_us)
{
    simulated_user_key.store(false);
    user_key_release_us.store(pressed && hold_us != 0 ? hal::SystemClock::now_us() + hold_us : 0);
    simulated_user_key.store(pressed);
}

void set_gamepad(std::uint8_t state, const std::array<float, 4>& sticks, const std::array<float, 2>& triggers,
                 std::uint16_t buttons)
{
    const core::CriticalSection lock;
    simulated_gamepad = {.connected = state != 0, .silent = state == 2, .sticks = sticks, .triggers = triggers,
                         .buttons = buttons};
}

void start_sim_gamepad(core::WorkQueue& queue)
{
    sim_gamepad.emplace(queue);
    sim_gamepad->start();
}

void start_vehicle(const estimation::WheelKinematics& kinematics,
                   const std::array<std::size_t, estimation::max_wheels>& outputs, float slip)
{
    vehicle.emplace(sim::GroundVehicle::Config{.kinematics = kinematics, .outputs = outputs, .slip = slip},
                    &hal::SystemClock::now_us, &wheel_speed);
    vehicle_imu.emplace(*vehicle, gyro_bias, mag_config);
    imu.set_output_source(&*vehicle_imu);
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

core::ResetReport reset_report()
{
    return last_reset;
}

void set_reset_report(const core::ResetReport& report)
{
    last_reset = report;
}

void raise_fault_when_driven(core::Fault fault)
{
    test_fault = fault;
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
                const std::uint64_t now = hal::SystemClock::now_us();
                if (watchdog->expired(now)) {
                    for (auto& output : motor_outputs) {
                        output.stop();
                    }
                }
                check_test_fault(now);
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

namespace vk::core {

void fault_stop(Fault fault, std::uint32_t address)
{
    // Only the running FreeRTOS task runs on the POSIX port, so nothing drives
    // the outputs again before the run ends.
    for (auto& output : platform::motor_outputs) {
        output.stop();
    }
    const std::string_view name = msg::enum_name(fault);
    std::printf("fault kind=%.*s address=0x%08x duty=", static_cast<int>(name.size()), name.data(),
                static_cast<unsigned>(address));
    for (std::size_t i = 0; i < platform::motor_outputs.size(); ++i) {
        std::printf(i == 0 ? "%.3f" : ",%.3f", static_cast<double>(platform::motor_outputs.at(i).duty()));
    }
    std::printf("\n");
    std::fflush(stdout);
    // At once, from whichever task faulted: FreeRTOS threads must not run exit handlers.
    std::_Exit(platform::fault_exit_status);
}

}  // namespace vk::core

extern "C" void vk_assert_failed()
{
    vk::core::fault_stop(vk::core::Fault::Assert,
                         static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(__builtin_return_address(0))));
}
