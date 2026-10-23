// SPDX-License-Identifier: GPL-3.0-or-later
// Firmware entry point: bring up the platform, start the drivers on their
// work queues, then hand over to FreeRTOS.
#include <board.hpp>
#include <vk/config.h>
#include <vk/core/critical_section.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/commander/commander_task.hpp>
#include <vk/params/params.hpp>
#include <vk/platform/gpio.hpp>
#include <vk/platform/platform.hpp>
#ifdef CONFIG_PARAM_STORAGE_FLASH
#include <vk/core/parameter_storage.hpp>
#include <vk/hal/flash.hpp>
#include <vk/platform/internal_flash.hpp>
#endif

#ifdef CONFIG_LINK_SERIAL
#include <vk/hal/uart.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/platform/dma_uart.hpp>
#endif

#ifdef CONFIG_ESTIMATOR_MAHONY
#include <vk/estimation/attitude_estimator.hpp>
#endif

#ifdef CONFIG_ACTUATOR_CONTROL
#include <vk/actuators/actuator_task.hpp>
#include <vk/hal/actuator.hpp>
#include <vk/platform/motor_output.hpp>
#endif

#ifdef CONFIG_DRIVER_ICM20948
#include <vk/drivers/icm20948.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/platform/soft_i2c.hpp>
#endif

#ifdef CONFIG_LOCAL_DISPLAY
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/local_display/local_display.hpp>
#include <vk/platform/soft_spi.hpp>
#endif
#ifdef CONFIG_LOCAL_MENU
#include <vk/local_display/local_menu_task.hpp>
#endif

#include <array>
#include <cstdint>
#include <string_view>

#ifndef CONFIG_PLATFORM_STM32F4
#error "the Kconfig layers must select the STM32F4 platform for firmware builds"
#endif

static_assert(vk::hal::distinct_pins<vk::board::pins::all>,
              "the board generator must never assign one pin to two roles");

#ifdef VK_FILL_FLASH_KB
// Size budget smoke test only: an image too large for the FIRMWARE region.
const unsigned char vk_flash_fill[VK_FILL_FLASH_KB * 1024] = {1};  // NOLINT
#endif

namespace {

#ifdef CONFIG_PARAM_STORAGE_FLASH
// Parameters live in sectors S1 and S2 of internal Flash (ADR-0020).
using ParamFlash = vk::platform::InternalFlash<vk::board::flash::PARAMS>;
static_assert(vk::hal::ParamFlash<ParamFlash>);
ParamFlash param_flash;
std::array<vk::core::ParameterStorage::Slot, vk::params::count> param_slots{};
vk::core::StaticInstance<vk::core::ParameterStorage> param_storage;

// Loads the stored parameters, before anything reads them, and starts storing
// changes. The boot erases the spare sector while still disarmed.
void start_parameters()
{
    param_storage.emplace(vk::params::parameters(), vk::hal::FlashSectors(param_flash), param_slots,
                          vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    param_storage->load();
    param_storage->start();
}
#endif

// The commander on wq:ctrl at the base rate (ADR-0005). The board counts as
// verified once the revision-ID pins have read the board's revision at boot
// and the IMU has answered (ADR-0010).
vk::core::StaticInstance<vk::commander::CommanderTask> commander;

bool revision_matches()
{
#ifdef CONFIG_BOARD_HAS_REVISION_ID
    using Revision = vk::platform::InputBits<vk::board::revision::pins>;
    Revision::init();
    return Revision::read() == vk::board::revision::value;
#else
    return true;
#endif
}

#ifdef CONFIG_BOARD_HAS_ACTUATOR_POWER_SENSE
using ActuatorPowerSense = vk::platform::InputLine<vk::board::actuator_power::pin>;

bool actuator_powered()
{
    return ActuatorPowerSense::is_high() == vk::board::actuator_power::powered_high;
}
#endif

constexpr std::uint32_t control_interval_us = 1'000'000 / vk::board::timing::base_rate_hz;

#ifdef CONFIG_ACTUATOR_CONTROL
// The board's four motor outputs (spec 0006).
static_assert(vk::board::actuators::count == 4, "the C50C entry point drives four motor outputs");
vk::platform::MotorOutputOn<vk::board::actuators::OUTPUT1> output1;
vk::platform::MotorOutputOn<vk::board::actuators::OUTPUT2> output2;
vk::platform::MotorOutputOn<vk::board::actuators::OUTPUT3> output3;
vk::platform::MotorOutputOn<vk::board::actuators::OUTPUT4> output4;
const std::array<vk::hal::MotorChannel, 4> motor_channels{{
    {.pwm = &output1, .direction = &output1, .encoder = &output1},
    {.pwm = &output2, .direction = &output2, .encoder = &output2},
    {.pwm = &output3, .direction = &output3, .encoder = &output3},
    {.pwm = &output4, .direction = &output4, .encoder = &output4},
}};
vk::core::StaticInstance<vk::actuators::ActuatorTask> actuator_control;

// Called from the output watchdog's interrupt.
void stop_motor_outputs()
{
    decltype(output1)::stop();
    decltype(output2)::stop();
    decltype(output3)::stop();
    decltype(output4)::stop();
}

// Actuator control on wq:ctrl after the commander, and the output watchdog.
void start_actuators()
{
    output1.init();
    output2.init();
    output3.init();
    output4.init();
#ifdef CONFIG_BRINGUP_MOTORS
    constexpr bool bringup = true;
#else
    constexpr bool bringup = false;
#endif
    actuator_control.emplace(vk::actuators::ActuatorTask::Config{.interval_us = control_interval_us,
                                                                 .channels = motor_channels,
                                                                 .bringup = bringup},
                             vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    actuator_control->start();
    vk::platform::start_output_watchdog(control_interval_us, &stop_motor_outputs);
}
#endif

void start_commander()
{
    vk::commander::CommanderTask::Config config{.interval_us = control_interval_us,
                                                .revision_ok = revision_matches(),
                                                .reset = &vk::platform::reset};
#ifdef CONFIG_ACTUATOR_CONTROL
    config.actuators_started = true;
    config.output_watchdog_trips = &vk::platform::output_watchdog_trips;
#endif
#ifdef CONFIG_BRINGUP_MOTORS
    config.bringup = true;
#endif
#ifdef CONFIG_BOARD_HAS_ACTUATOR_POWER_SENSE
    ActuatorPowerSense::init();
    config.actuator_power = &actuator_powered;
#endif
    commander.emplace(config, vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    commander->start();
}

#ifdef CONFIG_DRIVER_ICM20948
using Imu = vk::board::devices::IMU0;
static_assert(Imu::type == "ICM20948" && Imu::bus == "SOFT_I2C", "the C50C's IMU is an ICM20948 on software I2C");

// main()'s stack is reused once the scheduler runs, so long-lived objects are static.
vk::platform::SoftI2cOn<Imu::pins> imu_bus;
vk::core::StaticInstance<vk::drivers::Icm20948> imu;

void start_imu()
{
    imu_bus.init();
    imu.emplace(vk::hal::I2cDevice(imu_bus, Imu::address),
                vk::drivers::Icm20948::Config{
                    .instance = 0,
                    .rotation = Imu::rotation,
                    .board_orientation = static_cast<vk::hal::Rotation>(vk::params::sens.board_rot),
                    .rate_hz = vk::board::timing::base_rate_hz},
                vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    imu->start();
}
#endif

#ifdef CONFIG_LINK_SERIAL
// A Link on a board serial port, 8N1 at its SERIALn_BAUD (spec 0005), whose
// default the board definition gives. Each Link is a link_status instance and
// a source for setpoint arbitration (ADR-0005). The ports stay fixed until
// SERIALn_FUNCTION exists.
template <class Port>
struct SerialLink {
    using Uart = vk::platform::DmaUart<Port, vk::link::serial_transmit_buffer_size>;
    static_assert(vk::hal::Uart<Uart>);

    Uart uart;  // in .bss, normal SRAM, where the DMA reaches its buffers
    vk::core::StaticInstance<vk::hal::UartTransport> transport;
    vk::core::StaticInstance<vk::link::Link> link;

    void start(std::string_view name, std::uint8_t instance, std::int32_t baud_parameter, std::int32_t board_baud)
    {
        // A rate the USART cannot reach falls back to the board default, so a bad
        // parameter never costs the Link that could change it back.
        auto baud = static_cast<std::uint32_t>(baud_parameter);
        if (!uart.configure(baud)) {
            baud = static_cast<std::uint32_t>(board_baud);
            if (!uart.configure(baud)) {
                vk::core::fail("a Link's baud rate is out of the USART's reach");
            }
        }
        transport.emplace(uart, baud);
        link.emplace(name, *transport,
                     vk::link::default_topic_map(static_cast<float>(vk::board::timing::base_rate_hz), instance),
                     vk::link::Link::Config{.instance = instance,
                                            .parameters = {.parameters = &vk::params::parameters(),
                                                           .set_hash = vk::params::set_hash,
#ifdef CONFIG_PARAM_STORAGE_FLASH
                                                           .storage = &*param_storage,
#endif
                                                           },
                                            .control = {.control = &*commander}},
                     vk::core::work_queue(vk::core::WorkQueueTier::Io));
        link->start();
    }
};

// SERIAL1 is the Type-C port (USART3); SERIAL2 the one-key download port
// (USART1), which also flashes the board through the system bootloader.
static_assert(vk::board::defaults::SERIAL1_FUNCTION == "LINK" && vk::board::defaults::SERIAL2_FUNCTION == "LINK",
              "SERIAL1 and SERIAL2 must be Link ports");
SerialLink<vk::board::ports::SERIAL1> serial1_link;
SerialLink<vk::board::ports::SERIAL2> serial2_link;

void start_links()
{
    serial1_link.start("link0", 0, vk::params::serial.serial1_baud, vk::board::defaults::SERIAL1_BAUD);
    serial2_link.start("link1", 1, vk::params::serial.serial2_baud, vk::board::defaults::SERIAL2_BAUD);
}
#endif

#ifdef CONFIG_ESTIMATOR_MAHONY
vk::core::StaticInstance<vk::estimation::AttitudeEstimator> attitude_estimator;

// Right after the IMU on wq:ctrl, so each sample is estimated in the same pass (ADR-0008).
void start_estimator()
{
    attitude_estimator.emplace(vk::estimation::AttitudeEstimator::Config{},
                               vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    attitude_estimator->start();
}
#endif

#ifdef CONFIG_LOCAL_DISPLAY
using Display = vk::board::devices::DISPLAY0;
static_assert(Display::type == "SSD1306" && Display::bus == "SOFT_SPI",
              "the C50C's display is an SSD1306 on software SPI");

using DisplayBus = vk::platform::SoftSpiDisplayOn<Display::pins>;
static_assert(vk::hal::DisplayBus<DisplayBus>);

DisplayBus display_bus;
vk::core::StaticInstance<vk::drivers::Ssd1306> display_driver;
vk::core::StaticInstance<vk::local_display::LocalDisplay> local_display;

#ifdef CONFIG_LOCAL_MENU
using UserKey = vk::platform::InputLine<vk::board::user_key::pin>;
vk::core::StaticInstance<vk::local_display::LocalMenuTask> local_menu;

bool user_key_pressed()
{
    return UserKey::is_high() == vk::board::user_key::pressed_high;
}
#endif

// The local display on the board's OLED and the local menu on the user key,
// both on wq:slow (ADR-0022). Nothing reads the display back, so an unplugged
// OLED changes nothing else.
void start_local_display()
{
    const vk::local_display::LocalMenu* menu = nullptr;
#ifdef CONFIG_LOCAL_MENU
    UserKey::init();
    local_menu.emplace(vk::local_display::LocalMenuTask::Config{.key_pressed = &user_key_pressed},
                       vk::params::parameters(), vk::params::menu(), vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    local_menu->start();
    menu = &local_menu->menu();
#endif
    display_bus.init();
    display_driver.emplace(vk::hal::DisplayDevice(display_bus));
    local_display.emplace(*display_driver, menu, vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    local_display->start();
}
#endif

}  // namespace

int main()
{
#ifdef VK_FILL_FLASH_KB
    // Referenced so --gc-sections keeps the fill.
    __asm__ volatile("" : : "r"(vk_flash_fill));
#endif
    vk::platform::init();
#ifdef CONFIG_PARAM_STORAGE_FLASH
    start_parameters();
#endif
#ifdef CONFIG_DRIVER_ICM20948
    start_imu();
#endif
    start_commander();
#ifdef CONFIG_ACTUATOR_CONTROL
    start_actuators();
#endif
#ifdef CONFIG_ESTIMATOR_MAHONY
    start_estimator();
#endif
#ifdef CONFIG_LOCAL_DISPLAY
    start_local_display();
#endif
#ifdef CONFIG_LINK_SERIAL
    start_links();
#endif
    vk::core::start_scheduler();  // Never returns.
}
