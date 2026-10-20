#include <vk/platform/platform.hpp>

#include <vk/core/scheduler.hpp>
#include <vk/hal/clock.hpp>

#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/icm20948.hpp>
#include <vk/sim/motion.hpp>

#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <atomic>
#include <cstdint>

namespace vk::platform {
namespace {

StaticTask_t stop_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> stop_stack;
std::uint64_t stop_duration_us = 0;

std::atomic<bool> simulated_actuator_power{true};

sim::I2cBus bus;
sim::Icm20948 imu;
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

bool actuator_power()
{
    return simulated_actuator_power.load();
}

void set_actuator_power(bool powered)
{
    simulated_actuator_power.store(powered);
}

}  // namespace vk::platform
