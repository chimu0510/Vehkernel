#include <vk/sim/icm20948.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace vk::sim {
namespace {

// Bank 0.
constexpr std::uint8_t who_am_i = 0x00;
constexpr std::uint8_t pwr_mgmt_1 = 0x06;
constexpr std::uint8_t accel_xout_h = 0x2D;
constexpr std::uint8_t temp_out_l = 0x3A;
// Every bank.
constexpr std::uint8_t reg_bank_sel = 0x7F;
// Bank 2.
constexpr std::uint8_t gyro_config_1 = 0x01;
constexpr std::uint8_t accel_config = 0x14;

constexpr std::uint8_t device_reset = 0x80;
constexpr std::uint8_t sleep = 0x40;
constexpr float standard_gravity = 9.80665F;
constexpr float room_temp_offset_c = 21.0F;
constexpr float temp_sensitivity = 333.87F;  // LSB per degree Celsius

std::int16_t saturate(float value)
{
    const float rounded = std::round(value);
    return static_cast<std::int16_t>(std::clamp(rounded, -32768.0F, 32767.0F));
}

}  // namespace

Icm20948::Icm20948()
{
    reset();
    resets_ = 0;
}

void Icm20948::reset()
{
    for (auto& bank : banks_) {
        bank.fill(0);
    }
    banks_[0][who_am_i] = 0xEA;
    banks_[0][pwr_mgmt_1] = 0x41;  // Asleep, clock auto-selected.
    banks_[2][gyro_config_1] = 0x01;
    banks_[2][accel_config] = 0x01;
    bank_sel_ = 0;
    ++resets_;
}

std::uint8_t Icm20948::bank() const
{
    return static_cast<std::uint8_t>((bank_sel_ >> 4U) & 0x3U);
}

bool Icm20948::sleeping() const
{
    return (banks_[0][pwr_mgmt_1] & sleep) != 0;
}

std::uint8_t Icm20948::register_value(std::uint8_t bank, std::uint8_t reg) const
{
    return reg == reg_bank_sel ? bank_sel_ : banks_.at(bank).at(reg & 0x7FU);
}

bool Icm20948::write(std::span<const std::uint8_t> data)
{
    return responding_ && RegisterDevice::write(data);
}

bool Icm20948::read(std::span<std::uint8_t> data)
{
    if (!responding_) {
        return false;
    }
    latch_outputs();
    return RegisterDevice::read(data);
}

float Icm20948::noise(float amplitude)
{
    // xorshift32: deterministic, so SITL runs repeat exactly.
    noise_state_ ^= noise_state_ << 13U;
    noise_state_ ^= noise_state_ >> 17U;
    noise_state_ ^= noise_state_ << 5U;
    const float unit = static_cast<float>(noise_state_) / 4294967295.0F;  // [0, 1]
    return amplitude * (2.0F * unit - 1.0F);
}

void Icm20948::latch_outputs()
{
    if (sleeping()) {
        outputs_.fill(0);
        return;
    }
    if (source_ != nullptr) {
        source_->update(*this);
    }
    const auto accel_fs = static_cast<unsigned>((banks_[2][accel_config] >> 1U) & 0x3U);
    const auto gyro_fs = static_cast<unsigned>((banks_[2][gyro_config_1] >> 1U) & 0x3U);
    const float accel_lsb_per_g = 16384.0F / static_cast<float>(1U << accel_fs);
    const float gyro_lsb_per_dps = 131.0F / static_cast<float>(1U << gyro_fs);
    for (std::size_t i = 0; i < 3; ++i) {
        const float accel = accel_.at(i) + noise(accel_noise_);
        const float gyro = gyro_.at(i) + noise(gyro_noise_);
        outputs_.at(i) = saturate(accel / standard_gravity * accel_lsb_per_g);
        outputs_.at(i + 3) = saturate(gyro * 180.0F / std::numbers::pi_v<float> * gyro_lsb_per_dps);
    }
    outputs_[6] = saturate((temperature_ - room_temp_offset_c) * temp_sensitivity);
}

void Icm20948::write_register(std::uint8_t reg, std::uint8_t value)
{
    reg &= 0x7FU;
    if (reg == reg_bank_sel) {
        bank_sel_ = value & 0x30U;
        return;
    }
    if (bank() == 0 && reg == pwr_mgmt_1 && (value & device_reset) != 0) {
        reset();
        return;
    }
    if (bank() == 0 && reg == who_am_i) {
        return;  // Read-only.
    }
    banks_.at(bank()).at(reg) = value;
}

std::uint8_t Icm20948::read_register(std::uint8_t reg)
{
    reg &= 0x7FU;
    if (reg == reg_bank_sel) {
        return bank_sel_;
    }
    if (bank() == 0 && reg >= accel_xout_h && reg <= temp_out_l) {
        return output_register(reg);
    }
    return banks_.at(bank()).at(reg);
}

std::uint8_t Icm20948::output_register(std::uint8_t reg) const
{
    const auto offset = static_cast<std::size_t>(reg - accel_xout_h);
    const auto word = static_cast<std::uint16_t>(outputs_.at(offset / 2));
    return (offset % 2 == 0) ? static_cast<std::uint8_t>(word >> 8U) : static_cast<std::uint8_t>(word & 0xFFU);
}

}  // namespace vk::sim
