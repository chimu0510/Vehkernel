// Firmware entry point: bring up the platform, start the drivers on their
// work queues, then hand over to FreeRTOS.
#include <board.hpp>
#include <vk/config.h>
#include <vk/core/critical_section.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/params/params.hpp>
#include <vk/platform/platform.hpp>

#ifdef CONFIG_LINK_SERIAL
#include <vk/hal/uart.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/platform/dma_uart.hpp>
#endif

#ifdef CONFIG_ESTIMATOR_MAHONY
#include <vk/estimation/attitude_estimator.hpp>
#endif

#ifdef CONFIG_DRIVER_ICM20948
#include <vk/drivers/icm20948.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/platform/soft_i2c.hpp>
#endif

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
// The Link's serial port and baud rate, fixed until the parameter system
// exists (spec 0003): SERIAL1, the Type-C port, at 921600 baud 8N1.
using LinkPort = vk::board::ports::SERIAL1;
static_assert(vk::board::defaults::SERIAL1_FUNCTION == "LINK", "SERIAL1 must be a Link port");
constexpr std::uint32_t link_baud = 921'600;

using LinkUart = vk::platform::DmaUart<LinkPort>;
static_assert(vk::hal::Uart<LinkUart>);

LinkUart link_uart;  // in .bss, normal SRAM, where the DMA reaches its buffers
vk::core::StaticInstance<vk::hal::UartTransport> link_transport;
vk::core::StaticInstance<vk::link::Link> serial_link;

void start_link()
{
    if (!link_uart.configure(link_baud)) {
        vk::core::fail("the Link's baud rate is out of the USART's reach");
    }
    link_transport.emplace(link_uart);
    serial_link.emplace("link0", *link_transport, vk::link::default_topic_map(),
                        vk::link::Link::Config{.parameters = {.parameters = &vk::params::parameters(),
                                                              .set_hash = vk::params::set_hash}},
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
#ifdef CONFIG_DRIVER_ICM20948
    start_imu();
#endif
#ifdef CONFIG_ESTIMATOR_MAHONY
    start_estimator();
#endif
#ifdef CONFIG_LINK_SERIAL
    start_link();
#endif
    vk::core::start_scheduler();  // Never returns.
}
