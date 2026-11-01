// SPDX-License-Identifier: GPL-3.0-or-later
// Support code for STM32F4 board examples (spec 0013, ADR-0025).
#pragma once

#include <board.hpp>
#include <line.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/clock.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/soft_i2c.hpp>
#include <vk/platform/soft_spi.hpp>

#include <cstdint>
#include <string_view>

namespace example {

namespace detail {

using Display = vk::board::devices::DISPLAY0;
static_assert(Display::type == "SSD1306" && Display::bus == "SOFT_SPI",
              "the C50C's display is an SSD1306 on software SPI");

using DisplayBus = vk::platform::SoftSpiDisplayOn<Display::pins>;
static_assert(vk::hal::DisplayBus<DisplayBus>);

inline DisplayBus display_bus;
inline vk::core::StaticInstance<vk::drivers::Ssd1306> display_driver;

using Imu = vk::board::devices::IMU0;
static_assert(Imu::type == "ICM20948" && Imu::bus == "SOFT_I2C",
              "the C50C's IMU is an ICM20948 on software I2C");

inline vk::platform::SoftI2cOn<Imu::pins> imu_bus;
inline vk::core::StaticInstance<vk::drivers::Icm20948> imu_driver;

}  // namespace detail

// Brings up the platform: configures fault handlers, interrupt priorities,
// the clock tree from the board definition, the microsecond clock and the
// cycle counter (ADR-0009).
inline void init(int /*argc*/ = 0, char** /*argv*/ = nullptr)
{
    vk::platform::init();
}

// Starts the FreeRTOS scheduler (ADR-0004). Never returns.
inline void start_scheduler()
{
    vk::core::start_scheduler();
}

// Emits a key=value line in SITL for L3 testing (spec 0013); does nothing on
// the board.
template <typename T>
inline void report(std::string_view key, const T& value)
{
    static_cast<void>(key);
    static_cast<void>(value);
}

// Returns true on the board (runs indefinitely).
inline bool running()
{
    return true;
}

// Busy-waits until the system microsecond clock reaches target_us.
inline void sleep_until_us(std::uint64_t target_us)
{
    while (vk::hal::SystemClock::now_us() < target_us) {
    }
}

// Returns the board's SSD1306 OLED display driver on software SPI.
inline vk::drivers::Ssd1306& display()
{
    if (!detail::display_driver.has_value()) {
        detail::display_bus.init();
        detail::display_driver.emplace(vk::hal::DisplayDevice(detail::display_bus));
    }
    return *detail::display_driver;
}

// Returns the board's ICM20948 IMU driver on software I2C, running on wq:ctrl.
inline vk::drivers::Icm20948& imu()
{
    if (!detail::imu_driver.has_value()) {
        detail::imu_bus.init();
        detail::imu_driver.emplace(
            vk::hal::I2cDevice(detail::imu_bus, detail::Imu::address),
            vk::drivers::Icm20948::Config{
                .instance = 0,
                .rotation = detail::Imu::rotation,
                .board_orientation = vk::hal::Rotation::None,
                .rate_hz = vk::board::timing::base_rate_hz},
            vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    }
    return *detail::imu_driver;
}

}  // namespace example
