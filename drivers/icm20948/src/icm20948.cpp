#include <vk/drivers/icm20948.hpp>

#include <array>
#include <cstdint>
#include <numbers>

namespace vk::drivers {
namespace {

// Registers (datasheet DS-000189, section 7). Bank 0:
constexpr std::uint8_t who_am_i = 0x00;
constexpr std::uint8_t pwr_mgmt_1 = 0x06;
constexpr std::uint8_t pwr_mgmt_2 = 0x07;
constexpr std::uint8_t accel_xout_h = 0x2D;  // then ACCEL Y, Z, GYRO X, Y, Z, TEMP: 14 bytes
// Bank 2:
constexpr std::uint8_t gyro_smplrt_div = 0x00;
constexpr std::uint8_t gyro_config_1 = 0x01;
constexpr std::uint8_t accel_smplrt_div_1 = 0x10;
constexpr std::uint8_t accel_smplrt_div_2 = 0x11;
constexpr std::uint8_t accel_config = 0x14;
// Every bank:
constexpr std::uint8_t reg_bank_sel = 0x7F;

constexpr std::uint8_t device_reset = 0x80;
constexpr std::uint8_t clock_auto = 0x01;  // awake, best available clock

// Gyroscope: +-2000 dps, low-pass filter on at 119.5 Hz (GYRO_DLPFCFG = 3).
constexpr std::uint8_t gyro_config = (3U << 3U) | (3U << 1U) | 1U;
constexpr float gyro_lsb_per_dps = 16.4F;
// Accelerometer: +-16 g, low-pass filter on at 111.4 Hz (ACCEL_DLPFCFG = 3).
constexpr std::uint8_t accel_config_value = (3U << 3U) | (3U << 1U) | 1U;
constexpr float accel_lsb_per_g = 2048.0F;

constexpr float standard_gravity = 9.80665F;
constexpr float room_temp_offset_c = 21.0F;
constexpr float temp_sensitivity = 333.87F;  // LSB per degree Celsius
constexpr std::uint32_t internal_rate_hz = 1125;  // sample rate dividers divide this

constexpr std::uint64_t probe_retry_us = 100'000;
constexpr std::uint64_t reset_time_us = 100'000;
// Consecutive failed reads after which the device is probed and configured again.
constexpr std::uint32_t failures_before_reprobe = 20;

std::int16_t word_at(const std::array<std::uint8_t, 14>& raw, std::size_t index)
{
    return static_cast<std::int16_t>(static_cast<std::uint16_t>((raw.at(index) << 8U) | raw.at(index + 1)));
}

// Divider giving the lowest output data rate that is still at or above `rate_hz`.
std::uint8_t sample_rate_divider(std::uint32_t rate_hz)
{
    if (rate_hz == 0 || rate_hz >= internal_rate_hz) {
        return 0;
    }
    return static_cast<std::uint8_t>(internal_rate_hz / rate_hz - 1);
}

}  // namespace

Icm20948::Icm20948(const hal::I2cDevice& device, const Config& config, core::WorkQueue& queue)
    : core::WorkItem("icm20948", queue), device_(device), config_(config), publication_(config.instance)
{
}

void Icm20948::start()
{
    schedule_periodic(1'000'000 / config_.rate_hz);
}

Icm20948::Status Icm20948::status() const
{
    return {.state = state_,
            .samples = samples_,
            .read_failures = read_failures_,
            .probe_failures = probe_failures_,
            .last_who_am_i = last_who_am_i_};
}

void Icm20948::run()
{
    const std::uint64_t now = queue().now_us();
    switch (state_) {
    case State::Probing: probe(now); break;
    case State::Resetting: configure(now); break;
    case State::Running: read_sample(now); break;
    }
}

bool Icm20948::select_bank(std::uint8_t bank)
{
    return device_.write_register(reg_bank_sel, static_cast<std::uint8_t>(bank << 4U));
}

void Icm20948::probe(std::uint64_t now)
{
    if (now < next_probe_us_) {
        return;
    }
    std::array<std::uint8_t, 1> id{};
    const bool read = select_bank(0) && device_.read_register(who_am_i, id);
    last_who_am_i_ = read ? id[0] : 0;
    if (!read || id[0] != who_am_i_value) {
        ++probe_failures_;
        next_probe_us_ = now + probe_retry_us;
        return;
    }
    if (device_.write_register(pwr_mgmt_1, device_reset)) {
        reset_at_us_ = now;
        state_ = State::Resetting;
    }
}

void Icm20948::configure(std::uint64_t now)
{
    if (now - reset_at_us_ < reset_time_us) {
        return;
    }
    const std::uint8_t divider = sample_rate_divider(config_.rate_hz);
    std::array<std::uint8_t, 1> gyro_readback{};
    std::array<std::uint8_t, 1> accel_readback{};
    const bool ok = select_bank(0) && device_.write_register(pwr_mgmt_1, clock_auto) &&
                    device_.write_register(pwr_mgmt_2, 0x00) && select_bank(2) &&
                    device_.write_register(gyro_smplrt_div, divider) &&
                    device_.write_register(gyro_config_1, gyro_config) &&
                    device_.write_register(accel_smplrt_div_1, 0x00) &&
                    device_.write_register(accel_smplrt_div_2, divider) &&
                    device_.write_register(accel_config, accel_config_value) &&
                    device_.read_register(gyro_config_1, gyro_readback) &&
                    device_.read_register(accel_config, accel_readback) && select_bank(0);
    if (ok && gyro_readback[0] == gyro_config && accel_readback[0] == accel_config_value) {
        consecutive_failures_ = 0;
        state_ = State::Running;
    } else {
        ++probe_failures_;
        next_probe_us_ = now + probe_retry_us;
        state_ = State::Probing;
    }
}

void Icm20948::read_sample(std::uint64_t now)
{
    std::array<std::uint8_t, 14> raw{};
    if (!device_.read_register(accel_xout_h, raw)) {
        ++read_failures_;
        if (++consecutive_failures_ >= failures_before_reprobe) {
            state_ = State::Probing;
            next_probe_us_ = now;
        }
        return;
    }
    consecutive_failures_ = 0;

    constexpr float accel_scale = standard_gravity / accel_lsb_per_g;
    constexpr float gyro_scale = std::numbers::pi_v<float> / 180.0F / gyro_lsb_per_dps;
    // The sensor frame is x forward, y left, z up; the body frame is x
    // forward, y right, z down, so y and z change sign before the mounting
    // rotation is applied.
    const std::array<float, 3> accel{static_cast<float>(word_at(raw, 0)) * accel_scale,
                                     -static_cast<float>(word_at(raw, 2)) * accel_scale,
                                     -static_cast<float>(word_at(raw, 4)) * accel_scale};
    const std::array<float, 3> gyro{static_cast<float>(word_at(raw, 6)) * gyro_scale,
                                    -static_cast<float>(word_at(raw, 8)) * gyro_scale,
                                    -static_cast<float>(word_at(raw, 10)) * gyro_scale};

    msg::ImuSample sample{};
    sample.timestamp = now;
    sample.device_instance = config_.instance;
    sample.accel = hal::rotate(config_.rotation, accel);
    sample.gyro = hal::rotate(config_.rotation, gyro);
    sample.temperature = static_cast<float>(word_at(raw, 12)) / temp_sensitivity + room_temp_offset_c;
    publication_.publish(sample);
    ++samples_;
}

}  // namespace vk::drivers
