#include <vk/platform/platform.hpp>

#include <vk/core/scheduler.hpp>
#include <vk/hal/clock.hpp>

#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/icm20948.hpp>

#include <FreeRTOS.h>
#include <task.h>

#include <array>
#include <cstdint>

namespace vk::platform {
namespace {

StaticTask_t stop_tcb;
std::array<StackType_t, configMINIMAL_STACK_SIZE> stop_stack;
std::uint64_t stop_duration_us = 0;

sim::I2cBus bus;
sim::Icm20948 imu;

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

void init()
{
    // The first reading fixes the clock's zero at boot.
    static_cast<void>(hal::SystemClock::now_us());

    // Level and at rest, with sensor noise and a small gyroscope bias.
    imu.set_accel({0.0F, 0.0F, 9.80665F});
    imu.set_gyro({0.002F, -0.001F, 0.0005F});
    imu.set_temperature(30.0F);
    imu.set_noise(0.05F, 0.005F);
    bus.attach(imu_address, imu);
}

sim::I2cBus& imu_bus()
{
    return bus;
}

void stop_after(std::uint64_t duration_us)
{
    stop_duration_us = duration_us;
    xTaskCreateStatic(stop_task, "sitl_stop", stop_stack.size(), nullptr, configMAX_PRIORITIES - 1,
                      stop_stack.data(), &stop_tcb);
}

}  // namespace vk::platform
