// SPDX-License-Identifier: GPL-3.0-or-later
// Support code for STM32F4 board examples (spec 0013, ADR-0025).
#pragma once

#include <board.hpp>
#include <line.hpp>
#include <vk/core/critical_section.hpp>
#include <vk/core/parameter_storage.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/clock.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/hal/flash.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/hal/uart.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/params/params.hpp>
#include <vk/platform/dma_uart.hpp>
#include <vk/platform/internal_flash.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/soft_i2c.hpp>
#include <vk/platform/soft_spi.hpp>

#include <cstdint>
#include <string_view>
#include <type_traits>

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

using ParamFlash = vk::platform::InternalFlash<vk::board::flash::PARAMS>;
static_assert(vk::hal::ParamFlash<ParamFlash>);

inline ParamFlash param_flash;
inline std::array<vk::core::ParameterStorage::Slot, vk::params::count> param_slots{};
inline vk::core::StaticInstance<vk::core::ParameterStorage> param_storage;

template <class Port>
struct SerialLink {
    using Uart = vk::platform::DmaUart<Port, vk::link::serial_transmit_buffer_size>;
    static_assert(vk::hal::Uart<Uart>);

    Uart uart;
    vk::core::StaticInstance<vk::hal::UartTransport> transport;
    vk::core::StaticInstance<vk::link::Link> link;

    void start(std::string_view name, std::uint8_t instance, std::int32_t baud, vk::link::TopicMap map)
    {
        const auto baud_u32 = static_cast<std::uint32_t>(baud);
        if (!uart.configure(baud_u32)) {
            vk::core::fail("a Link's baud rate is out of the USART's reach");
        }
        transport.emplace(uart, baud_u32);
        link.emplace(name, *transport, map,
                     vk::link::Link::Config{
                         .instance = instance,
                         // Parameter access only once start_parameters() has loaded the stored set (09-parameters).
                         .parameters = param_storage.has_value()
                                 ? vk::link::ParameterAccess{.parameters = &vk::params::parameters(),
                                                             .set_hash = vk::params::set_hash,
                                                             .storage = &*param_storage}
                                 : vk::link::ParameterAccess{},
                     },
                     vk::core::work_queue(vk::core::WorkQueueTier::Io));
        link->start();
    }
};

inline SerialLink<vk::board::ports::SERIAL1> serial1_link;
inline SerialLink<vk::board::ports::SERIAL2> serial2_link;

template <typename MapFn>
vk::link::TopicMap get_topic_map(MapFn& fn, float imu_rate_hz, std::size_t link)
{
    if constexpr (std::is_invocable_v<MapFn, float, std::size_t>) {
        return fn(imu_rate_hz, link);
    } else if constexpr (std::is_invocable_v<MapFn, std::size_t>) {
        return fn(link);
    } else {
        return fn;
    }
}

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

// Loads the stored parameters from the PARAMS region at boot and starts storing changes.
inline void start_parameters()
{
    if (!detail::param_storage.has_value()) {
        detail::param_storage.emplace(
            vk::params::parameters(), vk::hal::FlashSectors(detail::param_flash), detail::param_slots,
            vk::core::work_queue(vk::core::WorkQueueTier::Slow));
        detail::param_storage->load();
        detail::param_storage->start();
    }
}

// Starts a Link on each board serial port configured for Link (SERIAL1, SERIAL2),
// using the board's default baud rates and DMA UARTs.
template <typename MapFn>
inline void start_links(MapFn&& map_fn)
{
    static_assert(vk::board::defaults::SERIAL1_FUNCTION == "LINK" && vk::board::defaults::SERIAL2_FUNCTION == "LINK",
                  "SERIAL1 and SERIAL2 must be Link ports");
    auto map0 = detail::get_topic_map(map_fn, static_cast<float>(vk::board::timing::base_rate_hz), 0);
    auto map1 = detail::get_topic_map(map_fn, static_cast<float>(vk::board::timing::base_rate_hz), 1);
    detail::serial1_link.start("link0", 0, vk::board::defaults::SERIAL1_BAUD, map0);
    detail::serial2_link.start("link1", 1, vk::board::defaults::SERIAL2_BAUD, map1);
}

// Returns one of the board's Links (instance 0 for SERIAL1, 1 for SERIAL2).
inline vk::link::Link& link(std::size_t instance = 0)
{
    if (instance == 0) {
        return *detail::serial1_link.link;
    }
    return *detail::serial2_link.link;
}

}  // namespace example
