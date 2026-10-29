// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/sim/icm20948.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace vk::sim {
namespace {

// Bank 0.
constexpr std::uint8_t who_am_i = 0x00;
constexpr std::uint8_t user_ctrl = 0x03;
constexpr std::uint8_t pwr_mgmt_1 = 0x06;
constexpr std::uint8_t i2c_mst_status = 0x17;
constexpr std::uint8_t accel_xout_h = 0x2D;
constexpr std::uint8_t temp_out_l = 0x3A;
constexpr std::uint8_t ext_slv_sens_data_00 = 0x3B;
constexpr std::uint8_t ext_slv_sens_data_08 = 0x43;
constexpr std::uint8_t fifo_en_1 = 0x66;
constexpr std::uint8_t fifo_en_2 = 0x67;
constexpr std::uint8_t fifo_rst = 0x68;
constexpr std::uint8_t fifo_mode = 0x69;
constexpr std::uint8_t fifo_counth = 0x70;
constexpr std::uint8_t fifo_countl = 0x71;
constexpr std::uint8_t fifo_r_w = 0x72;
// Every bank.
constexpr std::uint8_t reg_bank_sel = 0x7F;
// Bank 2.
constexpr std::uint8_t gyro_smplrt_div = 0x00;
constexpr std::uint8_t gyro_config_1 = 0x01;
constexpr std::uint8_t accel_config = 0x14;
// Bank 3.
constexpr std::uint8_t i2c_mst_ctrl = 0x01;
constexpr std::uint8_t i2c_slv0_addr = 0x03;
constexpr std::uint8_t i2c_slv0_reg = 0x04;
constexpr std::uint8_t i2c_slv0_ctrl = 0x05;
constexpr std::uint8_t i2c_slv4_addr = 0x13;
constexpr std::uint8_t i2c_slv4_reg = 0x14;
constexpr std::uint8_t i2c_slv4_ctrl = 0x15;
constexpr std::uint8_t i2c_slv4_do = 0x16;
constexpr std::uint8_t i2c_slv4_di = 0x17;

constexpr std::uint8_t ak09916_address = 0x0C;
constexpr std::uint8_t ak09916_who_am_i_reg = 0x01;
constexpr std::uint8_t ak09916_who_am_i_value = 0x09;
constexpr std::uint8_t ak09916_cntl2_reg = 0x31;
constexpr std::uint8_t ak09916_cntl3_reg = 0x32;
constexpr std::uint8_t ak09916_mode_cont_4 = 0x08;
constexpr std::uint64_t mag_interval_us = 10'000;  // 10 ms (100 Hz)
constexpr float mag_lsb = 0.15F;  // 0.15 µT per LSB

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

Icm20948::Icm20948(Clock clock) : clock_(clock)
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
    ak_cntl2_ = 0;
    mag_data_ready_ = false;
    mag_last_sample_us_ = 0;
    current_mag_raw_.fill(0);
    mag_bytes_.fill(0);
    fifo_.clear();
    last_master_cycle_us_ = 0;
    master_sub_us_ = 0;
    i2c_mst_status_ = 0;
    slv4_pending_ = false;
    mag_paused_ = false;
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

std::uint8_t Icm20948::ak09916_register_value(std::uint8_t reg) const
{
    switch (reg) {
    case 0x00: return 0x48;
    case 0x01: return ak09916_who_am_i_value;
    case 0x10: return mag_bytes_[0];
    case 0x11: return mag_bytes_[1];
    case 0x12: return mag_bytes_[2];
    case 0x13: return mag_bytes_[3];
    case 0x14: return mag_bytes_[4];
    case 0x15: return mag_bytes_[5];
    case 0x16: return mag_bytes_[6];
    case 0x17: return 0x00;
    case 0x18: return mag_bytes_[8];
    case 0x31: return ak_cntl2_;
    default: return 0;
    }
}

void Icm20948::push_fifo_byte(std::uint8_t byte)
{
    if (fifo_.size() >= 512) {
        fifo_.pop_front();
    }
    fifo_.push_back(byte);
}

void Icm20948::push_fifo(std::span<const std::uint8_t> bytes)
{
    for (const auto b : bytes) {
        push_fifo_byte(b);
    }
}

bool Icm20948::write(std::span<const std::uint8_t> data)
{
    if (!responding_) {
        return false;
    }
    advance_master_to(now_us());
    return RegisterDevice::write(data);
}

bool Icm20948::read(std::span<std::uint8_t> data)
{
    if (!responding_) {
        return false;
    }
    const std::uint8_t reg = pointer();
    const std::size_t len = data.size();
    ++read_counts_[{reg, len}];

    advance_master_to(now_us());
    if (bank() == 0 && pointer() == fifo_r_w) {
        for (auto& byte : data) {
            byte = read_register(fifo_r_w);
        }
        return true;
    }
    latch_outputs();
    return RegisterDevice::read(data);
}

std::vector<Icm20948::ReadTransfer> Icm20948::read_transfers() const
{
    std::vector<ReadTransfer> result;
    result.reserve(read_counts_.size());
    for (const auto& [key, count] : read_counts_) {
        result.push_back({.reg = key.first, .length = key.second, .count = count});
    }
    return result;
}

float Icm20948::noise(float amplitude)
{
    if (amplitude <= 0.0F) {
        return 0.0F;
    }
    // xorshift32: deterministic, so SITL runs repeat exactly.
    noise_state_ ^= noise_state_ << 13U;
    noise_state_ ^= noise_state_ >> 17U;
    noise_state_ ^= noise_state_ << 5U;
    const float unit = static_cast<float>(noise_state_) / 4294967295.0F;  // [0, 1]
    return amplitude * (2.0F * unit - 1.0F);
}

std::uint64_t Icm20948::now_us() const
{
    return clock_();
}

void Icm20948::advance_master_to(std::uint64_t now)
{
    if (last_master_cycle_us_ == 0) {
        last_master_cycle_us_ = now;
        return;
    }
    if (now <= last_master_cycle_us_) {
        return;
    }
    const std::uint8_t div = banks_[2][gyro_smplrt_div];
    const std::uint64_t period_num = static_cast<std::uint64_t>(1U + div) * 1'000'000ULL;
    constexpr std::uint64_t period_den = 1125ULL;

    while (true) {
        const std::uint64_t step_us = (master_sub_us_ + period_num) / period_den;
        const std::uint64_t next_cycle_us = last_master_cycle_us_ + step_us;
        if (next_cycle_us > now) {
            break;
        }
        master_sub_us_ = (master_sub_us_ + period_num) % period_den;
        last_master_cycle_us_ = next_cycle_us;
        run_master_cycle(next_cycle_us);
    }
}

void Icm20948::run_master_cycle(std::uint64_t cycle_time)
{
    const bool i2c_mst_en = (banks_[0][user_ctrl] & 0x20U) != 0;
    if (!i2c_mst_en) {
        return;
    }

    if (slv4_pending_) {
        slv4_pending_ = false;
        banks_[3][i2c_slv4_ctrl] &= ~0x80U;

        const std::uint8_t slv4_addr = banks_[3][i2c_slv4_addr];
        const bool is_read = (slv4_addr & 0x80U) != 0;
        const std::uint8_t addr = slv4_addr & 0x7FU;
        const std::uint8_t slv4_reg = banks_[3][i2c_slv4_reg];

        if (addr == ak09916_address) {
            if (!mag_responding_) {
                i2c_mst_status_ |= (0x40U | 0x10U);  // I2C_SLV4_DONE | I2C_SLV4_NACK
                banks_[3][i2c_slv4_di] = 0;
            } else {
                i2c_mst_status_ |= 0x40U;  // I2C_SLV4_DONE
                if (is_read) {
                    std::uint8_t data = 0;
                    switch (slv4_reg) {
                    case 0x00: data = 0x48; break;  // WIA1
                    case 0x01: data = ak09916_who_am_i_value; break;  // WIA2
                    case 0x10: data = mag_bytes_[0]; break;
                    case 0x11: data = mag_bytes_[1]; break;
                    case 0x12: data = mag_bytes_[2]; break;
                    case 0x13: data = mag_bytes_[3]; break;
                    case 0x14: data = mag_bytes_[4]; break;
                    case 0x15: data = mag_bytes_[5]; break;
                    case 0x16: data = mag_bytes_[6]; break;
                    case 0x17: data = 0x00; break;
                    case 0x18: data = mag_bytes_[8]; break;
                    case 0x31: data = ak_cntl2_; break;
                    default: data = 0; break;
                    }
                    banks_[3][i2c_slv4_di] = data;
                } else {
                    const std::uint8_t data = banks_[3][i2c_slv4_do];
                    if (slv4_reg == ak09916_cntl2_reg) {
                        ak_cntl2_ = data;
                        if (ak_cntl2_ == ak09916_mode_cont_4) {
                            mag_last_sample_us_ = cycle_time;
                            mag_data_ready_ = false;
                        } else if (ak_cntl2_ == 0x00U) {
                            mag_data_ready_ = false;
                        }
                    } else if (slv4_reg == ak09916_cntl3_reg) {
                        if ((data & 0x01U) != 0) {  // soft reset
                            ak_cntl2_ = 0;
                            mag_data_ready_ = false;
                        }
                    }
                }
            }
        } else {
            i2c_mst_status_ |= (0x40U | 0x10U);  // I2C_SLV4_DONE | I2C_SLV4_NACK
        }
    }

    const bool slv0_en = (banks_[3][i2c_slv0_ctrl] & 0x80U) != 0;
    const std::uint8_t slv0_addr = banks_[3][i2c_slv0_addr] & 0x7FU;
    if (slv0_en && slv0_addr == ak09916_address) {
        if (ak_cntl2_ == ak09916_mode_cont_4 && mag_responding_) {
            if (mag_paused_) {
                mag_last_sample_us_ = cycle_time;
            } else {
                if (mag_last_sample_us_ == 0) {
                    mag_last_sample_us_ = cycle_time;
                }
                while (cycle_time >= mag_last_sample_us_ + mag_interval_us) {
                    mag_last_sample_us_ += mag_interval_us;
                    mag_data_ready_ = true;

                    std::array<float, 3> f = field_;
                    if (source_ != nullptr) {
                        if (const auto dynamic_f = source_->field_at(mag_last_sample_us_)) {
                            f = *dynamic_f;
                        }
                    }

                    const float fx = f[0] + noise(mag_noise_);
                    const float fy = f[1] + noise(mag_noise_);
                    const float fz = f[2] + noise(mag_noise_);

                    // The AK09916's y and z point the other way relative to ICM20948's axes.
                    const float ak_x = fx;
                    const float ak_y = -fy;
                    const float ak_z = -fz;

                    const auto raw_x = saturate(ak_x / mag_lsb);
                    const auto raw_y = saturate(ak_y / mag_lsb);
                    const auto raw_z = saturate(ak_z / mag_lsb);

                    current_mag_raw_[0] = static_cast<std::uint8_t>(raw_x & 0xFFU);
                    current_mag_raw_[1] = static_cast<std::uint8_t>((raw_x >> 8U) & 0xFFU);
                    current_mag_raw_[2] = static_cast<std::uint8_t>(raw_y & 0xFFU);
                    current_mag_raw_[3] = static_cast<std::uint8_t>((raw_y >> 8U) & 0xFFU);
                    current_mag_raw_[4] = static_cast<std::uint8_t>(raw_z & 0xFFU);
                    current_mag_raw_[5] = static_cast<std::uint8_t>((raw_z >> 8U) & 0xFFU);
                }
            }
        }

        std::array<std::uint8_t, 9> record{};
        if (mag_responding_) {
            record[0] = mag_data_ready_ ? 0x01U : 0x00U;
            record[1] = current_mag_raw_[0];
            record[2] = current_mag_raw_[1];
            record[3] = current_mag_raw_[2];
            record[4] = current_mag_raw_[3];
            record[5] = current_mag_raw_[4];
            record[6] = current_mag_raw_[5];
            record[7] = 0x00U;
            record[8] = mag_overflow_ ? 0x08U : 0x00U;

            // Reading ST2 clears data ready, as on the device.
            mag_data_ready_ = false;
        }

        for (std::size_t i = 0; i < 9; ++i) {
            mag_bytes_[i] = record[i];
            banks_[0][ext_slv_sens_data_00 + i] = record[i];
        }

        const bool fifo_en = (banks_[0][user_ctrl] & 0x40U) != 0;
        const bool slv0_fifo_en = (banks_[0][fifo_en_1] & 0x01U) != 0;
        if (fifo_en && slv0_fifo_en) {
            for (std::size_t i = 0; i < 9; ++i) {
                push_fifo_byte(record[i]);
            }
        }
    }
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
    advance_master_to(now_us());
    reg &= 0x7FU;
    if (reg == reg_bank_sel) {
        bank_sel_ = value & 0x30U;
        return;
    }
    if (bank() == 0) {
        if (reg == pwr_mgmt_1 && (value & device_reset) != 0) {
            reset();
            return;
        }
        if (reg == who_am_i) {
            return;  // Read-only.
        }
        if (reg == fifo_rst) {
            banks_[0][fifo_rst] = value;
            if (value != 0) {
                fifo_.clear();
            }
            return;
        }
    }
    if (bank() == 3 && reg == i2c_slv4_ctrl) {
        banks_[3][i2c_slv4_ctrl] = value;
        if ((value & 0x80U) != 0) {
            slv4_pending_ = true;
        }
        return;
    }
    banks_.at(bank()).at(reg) = value;
}

std::uint8_t Icm20948::read_register(std::uint8_t reg)
{
    advance_master_to(now_us());
    reg &= 0x7FU;
    if (reg == reg_bank_sel) {
        return bank_sel_;
    }
    if (bank() == 0) {
        if (reg == i2c_mst_status) {
            const std::uint8_t val = i2c_mst_status_;
            i2c_mst_status_ = 0;
            return val;
        }
        if (reg == fifo_counth) {
            return static_cast<std::uint8_t>((fifo_.size() >> 8U) & 0xFFU);
        }
        if (reg == fifo_countl) {
            return static_cast<std::uint8_t>(fifo_.size() & 0xFFU);
        }
        if (reg == fifo_r_w) {
            if (fifo_.empty()) {
                return 0;
            }
            const std::uint8_t val = fifo_.front();
            fifo_.pop_front();
            return val;
        }
        if (reg >= accel_xout_h && reg <= ext_slv_sens_data_08) {
            return output_register(reg);
        }
    }
    return banks_.at(bank()).at(reg);
}

std::uint8_t Icm20948::output_register(std::uint8_t reg)
{
    if (reg <= temp_out_l) {
        const auto offset = static_cast<std::size_t>(reg - accel_xout_h);
        const auto word = static_cast<std::uint16_t>(outputs_.at(offset / 2));
        return (offset % 2 == 0) ? static_cast<std::uint8_t>(word >> 8U) : static_cast<std::uint8_t>(word & 0xFFU);
    }
    if (reg >= ext_slv_sens_data_00 && reg <= ext_slv_sens_data_08) {
        return banks_[0][reg];
    }
    return 0;
}

}  // namespace vk::sim
