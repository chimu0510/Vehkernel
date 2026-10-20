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

#include <array>
#include <cstdint>

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
                                                .revision_ok = revision_matches()};
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
// The Link's serial port: SERIAL1, the Type-C port, 8N1 at SERIAL1_BAUD
// (spec 0005), whose default the board definition gives. The port stays
// fixed until SERIALn_FUNCTION exists.
using LinkPort = vk::board::ports::SERIAL1;
static_assert(vk::board::defaults::SERIAL1_FUNCTION == "LINK", "SERIAL1 must be a Link port");

using LinkUart = vk::platform::DmaUart<LinkPort>;
static_assert(vk::hal::Uart<LinkUart>);

LinkUart link_uart;  // in .bss, normal SRAM, where the DMA reaches its buffers
vk::core::StaticInstance<vk::hal::UartTransport> link_transport;
vk::core::StaticInstance<vk::link::Link> serial_link;

void start_link()
{
    // A rate the USART cannot reach falls back to the board default, so a bad
    // parameter never costs the Link that could change it back.
    auto baud = static_cast<std::uint32_t>(vk::params::serial.serial1_baud);
    if (!link_uart.configure(baud)) {
        baud = static_cast<std::uint32_t>(vk::board::defaults::SERIAL1_BAUD);
        if (!link_uart.configure(baud)) {
            vk::core::fail("the Link's baud rate is out of the USART's reach");
        }
    }
    link_transport.emplace(link_uart, baud);
    serial_link.emplace("link0", *link_transport,
                        vk::link::default_topic_map(static_cast<float>(vk::board::timing::base_rate_hz)),
                        vk::link::Link::Config{.parameters = {.parameters = &vk::params::parameters(),
                                                              .set_hash = vk::params::set_hash,
#ifdef CONFIG_PARAM_STORAGE_FLASH
                                                              .storage = &*param_storage,
#endif
                                                              },
                                               .control = {.control = &*commander}},
                        vk::core::work_queue(vk::core::WorkQueueTier::Io));
    serial_link->start();
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
#ifdef CONFIG_LINK_SERIAL
    start_link();
#endif
    vk::core::start_scheduler();  // Never returns.
}
