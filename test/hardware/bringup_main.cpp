// Bring-up test build (ADR-0018, L4): runs the bring-up cases once at boot,
// writes the results to vk_bringup_results and idles. The bring-up runner on
// the development host flashes this image over SWD and reads the block back.
//
// Safe with motors connected: only the clock tree, TIM7, the cycle counter
// and the IMU's software I2C pins are configured; actuator outputs keep their
// reset state (inputs).
#include "bringup_results.h"

#include <vk/config.h>
#include <vk/hal/clock.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/soft_i2c.hpp>

#include <board.hpp>

#include <algorithm>
#include <array>
#include <cstdint>

#if !defined(CONFIG_BRINGUP_TESTS) || !defined(CONFIG_SOFT_I2C)
#error "the bring-up build needs CONFIG_BRINGUP_TESTS and CONFIG_SOFT_I2C"
#endif

// Read by the runner over SWD; the name is part of the runner's contract.
extern "C" {
[[gnu::used]] volatile vk_bringup_results vk_bringup_results;
}

namespace {

namespace board = vk::board;
using Imu = board::devices::IMU0;

static_assert(Imu::type == "ICM20948" && Imu::bus == "SOFT_I2C", "these cases are written for an ICM20948 on software I2C");

template <class List>
struct I2cPins;
template <class Scl, class Sda>
struct I2cPins<vk::hal::PinList<Scl, Sda>> {
    using Bus = vk::platform::SoftI2c<Scl, Sda>;
};
using ImuBus = I2cPins<Imu::pins>::Bus;
static_assert(vk::hal::I2cBus<ImuBus>);

// ICM20948 registers in user bank 0 (datasheet DS-000189, section 7).
constexpr std::uint8_t reg_who_am_i = 0x00;
constexpr std::uint8_t reg_accel_xout_h = 0x2D;  // ACCEL, GYRO and TEMP follow: 14 bytes
constexpr std::uint8_t reg_bank_sel = 0x7F;
constexpr std::uint8_t who_am_i_expected = 0xEA;
constexpr std::size_t sample_bytes = 14;
constexpr int rate_reads = 1000;

ImuBus imu_bus;

void record(std::size_t slot, vk_bringup_case_id id, bool pass, std::array<std::int32_t, VK_BRINGUP_VALUES> values)
{
    volatile vk_bringup_case& c = vk_bringup_results.cases[slot];
    c.id = id;
    for (std::size_t i = 0; i < values.size(); ++i) {
        c.value[i] = values.at(i);
    }
    c.status = pass ? VK_BRINGUP_PASS : VK_BRINGUP_FAIL;
}

void case_who_am_i(const vk::hal::I2cDevice& imu, std::size_t slot)
{
    // Select bank 0 in case an earlier firmware left another bank selected.
    static_cast<void>(imu.write_register(reg_bank_sel, 0));
    std::array<std::uint8_t, 1> id{};
    const bool acked = imu.read_register(reg_who_am_i, id);
    record(slot, VK_BRINGUP_IMU_WHO_AM_I, acked && id[0] == who_am_i_expected, {id[0], acked ? 1 : 0, 0, 0});
}

void case_read_rate(const vk::hal::I2cDevice& imu, std::size_t slot)
{
    std::array<std::uint8_t, sample_bytes> sample{};
    std::uint64_t total_us = 0;
    std::uint64_t longest_us = 0;
    std::int32_t failures = 0;
    for (int i = 0; i < rate_reads; ++i) {
        const std::uint64_t start = vk::hal::SystemClock::now_us();
        const bool ok = imu.read_register(reg_accel_xout_h, sample);
        const std::uint64_t elapsed = vk::hal::SystemClock::now_us() - start;
        total_us += elapsed;
        longest_us = std::max(longest_us, elapsed);
        failures += ok ? 0 : 1;
    }
    const auto mean_ns = static_cast<std::int32_t>(total_us * 1000 / rate_reads);
    const std::int32_t rate_hz = mean_ns > 0 ? 1'000'000'000 / mean_ns : 0;
    // Passes when every read was acknowledged and the timing preset's rate is reachable.
    const bool pass = failures == 0 && rate_hz >= static_cast<std::int32_t>(board::timing::base_rate_hz);
    record(slot, VK_BRINGUP_IMU_READ_RATE, pass,
           {mean_ns, static_cast<std::int32_t>(longest_us * 1000), rate_hz, failures});
}

}  // namespace

int main()
{
    vk::platform::init();

    vk_bringup_results.complete = 0;
    vk_bringup_results.case_count = 2;
    vk_bringup_results.version = VK_BRINGUP_VERSION;
    vk_bringup_results.magic = VK_BRINGUP_MAGIC;

    imu_bus.init();
    const vk::hal::I2cDevice imu(imu_bus, Imu::address);
    case_who_am_i(imu, 0);
    case_read_rate(imu, 1);

    vk_bringup_results.complete = 1;
    for (;;) {
        __asm__ volatile("wfi");
    }
}
