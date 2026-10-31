// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/drivers/icm20948.hpp>

#if defined(CONFIG_DRIVER_ICM20948_MAG)
#include <vk/params/params.hpp>
#endif

#include <array>
#include <cstdint>
#include <numbers>
#include <span>

namespace vk::drivers {
namespace {

// Registers (datasheet DS-000189, section 7). Bank 0:
constexpr std::uint8_t who_am_i = 0x00;
constexpr std::uint8_t user_ctrl = 0x03;
constexpr std::uint8_t lp_config = 0x05;
constexpr std::uint8_t pwr_mgmt_1 = 0x06;
constexpr std::uint8_t pwr_mgmt_2 = 0x07;
constexpr std::uint8_t i2c_mst_status = 0x17;
constexpr std::uint8_t accel_xout_h = 0x2D;  // ACCEL Y, Z, GYRO X, Y, Z, TEMP: 14 bytes
constexpr std::uint8_t fifo_en_1 = 0x66;
constexpr std::uint8_t fifo_en_2 = 0x67;
constexpr std::uint8_t fifo_rst = 0x68;
constexpr std::uint8_t fifo_mode = 0x69;
constexpr std::uint8_t fifo_counth = 0x70;
constexpr std::uint8_t fifo_countl = 0x71;
constexpr std::uint8_t fifo_r_w = 0x72;
// Bank 2:
constexpr std::uint8_t gyro_smplrt_div = 0x00;
constexpr std::uint8_t gyro_config_1 = 0x01;
constexpr std::uint8_t accel_smplrt_div_1 = 0x10;
constexpr std::uint8_t accel_smplrt_div_2 = 0x11;
constexpr std::uint8_t accel_config = 0x14;
// Bank 3:
constexpr std::uint8_t i2c_mst_odr_config = 0x00;
constexpr std::uint8_t i2c_mst_ctrl = 0x01;
constexpr std::uint8_t i2c_slv0_addr = 0x03;
constexpr std::uint8_t i2c_slv0_reg = 0x04;
constexpr std::uint8_t i2c_slv0_ctrl = 0x05;
constexpr std::uint8_t i2c_slv4_addr = 0x13;
constexpr std::uint8_t i2c_slv4_reg = 0x14;
constexpr std::uint8_t i2c_slv4_ctrl = 0x15;
constexpr std::uint8_t i2c_slv4_do = 0x16;
constexpr std::uint8_t i2c_slv4_di = 0x17;
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

#if defined(CONFIG_DRIVER_ICM20948_MAG)
// AK09916 magnetometer via internal I2C master and FIFO (spec 0012).
constexpr std::uint8_t user_ctrl_fifo_en = 0x40;
constexpr std::uint8_t user_ctrl_i2c_mst_en = 0x20;
constexpr std::uint8_t i2c_slv4_done = 0x40;
constexpr std::uint8_t i2c_slv4_nack = 0x10;
constexpr std::uint8_t slv_0_fifo_en = 0x01;
constexpr std::uint8_t i2c_mst_clk_400khz = 0x07;
// The master runs only while LP_CONFIG's I2C_MST_CYCLE is set, its reset value; it
// then reads its slaves at 1125 / 2^I2C_MST_ODR_CONFIG Hz, a division of the clock the
// gyroscope's sample rate divider divides too. A read between two FIFO writes clears
// the AK09916's DRDY unrecorded, so the master runs at the gyroscope's output data
// rate: with equal periods one read falls between two FIFO writes, whatever their
// phase (#179).
constexpr std::uint8_t lp_config_i2c_mst_cycle = 0x40;
constexpr std::uint8_t i2c_slv0_en = 0x80;
constexpr std::uint8_t i2c_slv4_en = 0x80;
constexpr std::uint8_t ak09916_address = 0x0C;
constexpr std::uint8_t ak09916_who_am_i_reg = 0x01;
constexpr std::uint8_t ak09916_who_am_i_value = 0x09;
constexpr std::uint8_t ak09916_cntl2_reg = 0x31;
constexpr std::uint8_t ak09916_mode_cont_4 = 0x08;  // 100 Hz continuous mode 4
constexpr std::uint8_t ak09916_st1_reg = 0x10;
constexpr std::uint8_t ak09916_read_len = 0x09;     // 9 bytes: ST1 to ST2
constexpr std::uint8_t ak09916_st1_drdy = 0x01;
constexpr std::uint8_t ak09916_st2_hofl = 0x08;
constexpr float mag_lsb_uT = 0.15F;  // 0.15 µT per LSB

constexpr std::uint64_t mag_step_timeout_us = 20'000;   // 20 ms for slave 4 steps
constexpr std::uint64_t mag_stale_timeout_us = 100'000; // 100 ms without data ready
constexpr std::uint64_t mag_status_period_us = 1'000'000; // magnetometer_status once a second
// FIFO records read in one run; more than this resets the FIFO.
constexpr std::size_t max_mag_fifo_bytes = std::size_t{4} * ak09916_read_len;
#endif

std::int16_t be_word_at(std::span<const std::uint8_t> raw, std::size_t index)
{
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>((raw[index] << 8U) | raw[index + 1]));
}

#if defined(CONFIG_DRIVER_ICM20948_MAG)
std::int16_t le_word_at(std::span<const std::uint8_t> raw, std::size_t index)
{
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(raw[index] | (raw[index + 1] << 8U)));
}
#endif

#if !defined(CONFIG_DRIVER_ICM20948_MAG)
// Divider giving the lowest output data rate that is still at or above `rate_hz`.
std::uint8_t sample_rate_divider(std::uint32_t rate_hz)
{
    if (rate_hz == 0 || rate_hz >= internal_rate_hz) {
        return 0;
    }
    return static_cast<std::uint8_t>(internal_rate_hz / rate_hz - 1);
}
#else
// I2C_MST_ODR_CONFIG giving the lowest master rate, 1125 / 2^n Hz, still at or above
// `rate_hz`; the gyroscope's divider is then 2^n - 1, so both run at that rate.
std::uint8_t master_odr_exponent(std::uint32_t rate_hz)
{
    constexpr std::uint8_t max_exponent = 8;  // 2^8 - 1 is the largest 8-bit divider
    std::uint8_t n = 0;
    while (rate_hz > 0 && n < max_exponent && (internal_rate_hz >> (n + 1U)) >= rate_hz) {
        ++n;
    }
    return n;
}
#endif

}  // namespace

Icm20948::Icm20948(const hal::I2cDevice& device, const Config& config, core::WorkQueue& queue)
    : core::WorkItem("icm20948", queue), device_(device), config_(config),
      to_body_(hal::compose(config.rotation, config.board_orientation)), publication_(config.instance)
#if defined(CONFIG_DRIVER_ICM20948_MAG)
      , mag_publication_(config.instance), mag_status_publication_(config.instance)
#endif
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
            .last_who_am_i = last_who_am_i_
#if defined(CONFIG_DRIVER_ICM20948_MAG)
            , .mag_samples = mag_samples_,
            .mag_failures = mag_failures_,
            .mag_overflows = mag_overflows_,
            .mag_configured = (mag_state_ == MagState::Running)
#endif
    };
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

#if defined(CONFIG_DRIVER_ICM20948_MAG)
void Icm20948::reset_mag_fifo()
{
    // A failed reset shows as another bad count on the next run.
    static_cast<void>(device_.write_register(fifo_rst, 0x1FU));
    static_cast<void>(device_.write_register(fifo_rst, 0x00U));
}

void Icm20948::count_mag_failure(MagFailure reason)
{
    ++mag_failures_;
    mag_last_failure_ = reason;
}

void Icm20948::reset_mag_setup(std::uint64_t now, MagFailure reason)
{
    select_bank(0);
    count_mag_failure(reason);
    next_mag_probe_us_ = now + probe_retry_us;
    mag_state_ = MagState::Probing;
}

void Icm20948::read_mag_fifo(std::uint64_t now)
{
    std::array<std::uint8_t, 2> count_raw{};
    if (!device_.read_register(fifo_counth, count_raw)) {
        count_mag_failure(MagFailure::Bus);
        return;
    }
    const std::uint16_t count = (static_cast<std::uint16_t>(count_raw[0]) << 8U) | count_raw[1];
    mag_fifo_count_ = count;
    if (count == 0) {
        return;
    }
    // A count that is not a multiple of 9 can be read while a record is being
    // written, so the FIFO is read again on the next run. A count that is still
    // not a multiple of 9 then, or more than 4 records (36 bytes), resets the
    // FIFO and counts a magnetometer failure.
    const bool partial = (count % ak09916_read_len) != 0;
    if ((partial && mag_fifo_partial_) || count > max_mag_fifo_bytes) {
        mag_fifo_partial_ = false;
        reset_mag_fifo();
        count_mag_failure(MagFailure::BadFifoCount);
        return;
    }
    mag_fifo_partial_ = partial;
    if (partial) {
        return;
    }

    std::array<std::uint8_t, max_mag_fifo_bytes> fifo_buf{};
    auto record_span = std::span{fifo_buf}.first(count);
    if (!device_.read_register(fifo_r_w, record_span)) {
        count_mag_failure(MagFailure::Bus);
        return;
    }

    const std::size_t num_records = count / ak09916_read_len;
    for (std::size_t r = 0; r < num_records; ++r) {
        const auto record = record_span.subspan(r * ak09916_read_len, ak09916_read_len);
        const std::uint8_t st1 = record[0];
        if ((st1 & ak09916_st1_drdy) != 0) {
            last_mag_sample_us_ = now;
            const std::uint8_t st2 = record[8];
            if ((st2 & ak09916_st2_hofl) != 0) {
                ++mag_overflows_;
            } else {
                const auto mag_x_raw = le_word_at(record, 1);
                const auto mag_y_raw = le_word_at(record, 3);
                const auto mag_z_raw = le_word_at(record, 5);

                const float ak_x = static_cast<float>(mag_x_raw) * mag_lsb_uT;
                const float ak_y = static_cast<float>(mag_y_raw) * mag_lsb_uT;
                const float ak_z = static_cast<float>(mag_z_raw) * mag_lsb_uT;

                // The AK09916 datasheet places its y and z axes opposite to the ICM20948's
                // sensor frame; the sensor frame's y and z axes are in turn opposite to the
                // board frame (x forward, y right, z down), so the two sign changes cancel:
                // the AK09916 axes already point in the board frame (ak_x forward, ak_y right,
                // ak_z down). Then to_body_ (composed of mounting rotation and board
                // orientation SENS_BOARD_ROT) turns the board frame into the body frame.
                const std::array<float, 3> unrotated{ak_x, ak_y, ak_z};
                const std::array<float, 3> body = to_body_ * unrotated;

                const auto& cal = params::cal;
                const std::array<float, 3> calibrated{
                    (body[0] - cal.mag_xoff) * cal.mag_xscale,
                    (body[1] - cal.mag_yoff) * cal.mag_yscale,
                    (body[2] - cal.mag_zoff) * cal.mag_zscale,
                };

                msg::MagnetometerSample sample{};
                sample.timestamp = now;
                sample.device_instance = config_.instance;
                sample.field = calibrated;
                sample.overflows = mag_overflows_;
                sample.failures = mag_failures_;
                mag_publication_.publish(sample);
                ++mag_samples_;
            }
        }
    }
}

void Icm20948::service_magnetometer(std::uint64_t now)
{
    switch (mag_state_) {
    case MagState::Probing:
        if (now >= next_mag_probe_us_) {
            if (!select_bank(0) || !device_.write_register(lp_config, lp_config_i2c_mst_cycle) ||
                !device_.write_register(user_ctrl, user_ctrl_i2c_mst_en)) {
                reset_mag_setup(now, MagFailure::Bus);
                return;
            }
            if (!select_bank(3) ||
                !device_.write_register(i2c_mst_odr_config, master_odr_exponent(config_.rate_hz)) ||
                !device_.write_register(i2c_mst_ctrl, i2c_mst_clk_400khz) ||
                !device_.write_register(i2c_slv4_addr, ak09916_address | 0x80U) ||
                !device_.write_register(i2c_slv4_reg, ak09916_who_am_i_reg) ||
                !device_.write_register(i2c_slv4_ctrl, i2c_slv4_en)) {
                reset_mag_setup(now, MagFailure::Bus);
                return;
            }
            select_bank(0);
            mag_step_started_us_ = now;
            mag_state_ = MagState::WaitingWhoAmI;
        }
        break;

    case MagState::WaitingWhoAmI: {
        std::array<std::uint8_t, 1> status_raw{};
        if (!device_.read_register(i2c_mst_status, status_raw)) {
            reset_mag_setup(now, MagFailure::Bus);
            return;
        }
        const std::uint8_t mst_status = status_raw[0];
        mag_mst_status_ = mst_status;
        if ((mst_status & i2c_slv4_nack) != 0) {
            reset_mag_setup(now, MagFailure::Nack);
            return;
        }
        if ((mst_status & i2c_slv4_done) != 0) {
            std::array<std::uint8_t, 1> wia{};
            if (!select_bank(3) || !device_.read_register(i2c_slv4_di, wia)) {
                reset_mag_setup(now, MagFailure::Bus);
                return;
            }
            mag_who_am_i_ = wia[0];
            if (wia[0] != ak09916_who_am_i_value) {
                reset_mag_setup(now, MagFailure::WrongWhoAmI);
                return;
            }
            if (!device_.write_register(i2c_slv4_addr, ak09916_address) ||
                !device_.write_register(i2c_slv4_reg, ak09916_cntl2_reg) ||
                !device_.write_register(i2c_slv4_do, ak09916_mode_cont_4) ||
                !device_.write_register(i2c_slv4_ctrl, i2c_slv4_en)) {
                reset_mag_setup(now, MagFailure::Bus);
                return;
            }
            select_bank(0);
            mag_step_started_us_ = now;
            mag_state_ = MagState::WaitingCntl2;
        } else if (now - mag_step_started_us_ >= mag_step_timeout_us) {
            reset_mag_setup(now, MagFailure::Timeout);
        }
        break;
    }

    case MagState::WaitingCntl2: {
        std::array<std::uint8_t, 1> status_raw{};
        if (!device_.read_register(i2c_mst_status, status_raw)) {
            reset_mag_setup(now, MagFailure::Bus);
            return;
        }
        const std::uint8_t mst_status = status_raw[0];
        mag_mst_status_ = mst_status;
        if ((mst_status & i2c_slv4_nack) != 0) {
            reset_mag_setup(now, MagFailure::Nack);
            return;
        }
        if ((mst_status & i2c_slv4_done) != 0) {
            if (!select_bank(3) ||
                !device_.write_register(i2c_slv0_addr, ak09916_address | 0x80U) ||
                !device_.write_register(i2c_slv0_reg, ak09916_st1_reg) ||
                !device_.write_register(i2c_slv0_ctrl, i2c_slv0_en | ak09916_read_len)) {
                reset_mag_setup(now, MagFailure::Bus);
                return;
            }
            if (!select_bank(0) ||
                !device_.write_register(fifo_rst, 0x1FU) ||
                !device_.write_register(fifo_rst, 0x00U) ||
                !device_.write_register(fifo_en_1, slv_0_fifo_en) ||
                !device_.write_register(fifo_en_2, 0x00U) ||
                !device_.write_register(fifo_mode, 0x00U) ||
                !device_.write_register(user_ctrl, user_ctrl_fifo_en | user_ctrl_i2c_mst_en)) {
                reset_mag_setup(now, MagFailure::Bus);
                return;
            }
            last_mag_sample_us_ = now;
            ++mag_setups_;
            mag_fifo_partial_ = false;
            mag_state_ = MagState::Running;
        } else if (now - mag_step_started_us_ >= mag_step_timeout_us) {
            reset_mag_setup(now, MagFailure::Timeout);
        }
        break;
    }

    case MagState::Running:
        read_mag_fifo(now);
        if (now - last_mag_sample_us_ >= mag_stale_timeout_us) {
            reset_mag_setup(now, MagFailure::Stale);
        }
        break;
    }
}

void Icm20948::publish_mag_status(std::uint64_t now)
{
    if (now < next_mag_status_us_) {
        return;
    }
    next_mag_status_us_ = now + mag_status_period_us;
    msg::MagnetometerStatus status{};
    status.timestamp = now;
    status.device_instance = config_.instance;
    status.state = static_cast<std::uint8_t>(mag_state_);
    status.last_failure = static_cast<std::uint8_t>(mag_last_failure_);
    status.i2c_mst_status = mag_mst_status_;
    status.who_am_i = mag_who_am_i_;
    status.fifo_count = mag_fifo_count_;
    status.samples = mag_samples_;
    status.failures = mag_failures_;
    status.overflows = mag_overflows_;
    status.setups = mag_setups_;
    mag_status_publication_.publish(status);
}
#endif

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
#if defined(CONFIG_DRIVER_ICM20948_MAG)
        mag_state_ = MagState::Probing;
        next_mag_probe_us_ = now;
#endif
    }
}

void Icm20948::configure(std::uint64_t now)
{
    if (now - reset_at_us_ < reset_time_us) {
        return;
    }
#if defined(CONFIG_DRIVER_ICM20948_MAG)
    // The I2C master's rate, so the FIFO records every magnetometer read.
    const auto divider = static_cast<std::uint8_t>((1U << master_odr_exponent(config_.rate_hz)) - 1U);
#else
    const std::uint8_t divider = sample_rate_divider(config_.rate_hz);
#endif
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
#if defined(CONFIG_DRIVER_ICM20948_MAG)
        mag_state_ = MagState::Probing;
        next_mag_probe_us_ = now;
#endif
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
#if defined(CONFIG_DRIVER_ICM20948_MAG)
            mag_state_ = MagState::Probing;
            next_mag_probe_us_ = now;
#endif
        }
        return;
    }
    consecutive_failures_ = 0;

    constexpr float accel_scale = standard_gravity / accel_lsb_per_g;
    constexpr float gyro_scale = std::numbers::pi_v<float> / 180.0F / gyro_lsb_per_dps;
    // The sensor frame is x forward, y left, z up; the body frame is x
    // forward, y right, z down, so y and z change sign before the mounting
    // rotation and the board orientation are applied.
    const std::array<float, 3> accel{static_cast<float>(be_word_at(raw, 0)) * accel_scale,
                                     -static_cast<float>(be_word_at(raw, 2)) * accel_scale,
                                     -static_cast<float>(be_word_at(raw, 4)) * accel_scale};
    const std::array<float, 3> gyro{static_cast<float>(be_word_at(raw, 6)) * gyro_scale,
                                    -static_cast<float>(be_word_at(raw, 8)) * gyro_scale,
                                    -static_cast<float>(be_word_at(raw, 10)) * gyro_scale};

    msg::ImuSample sample{};
    sample.timestamp = now;
    sample.device_instance = config_.instance;
    sample.accel = to_body_ * accel;
    sample.gyro = to_body_ * gyro;
    sample.temperature = static_cast<float>(be_word_at(raw, 12)) / temp_sensitivity + room_temp_offset_c;
    publication_.publish(sample);
    ++samples_;

#if defined(CONFIG_DRIVER_ICM20948_MAG)
    service_magnetometer(now);
    publish_mag_status(now);
#endif
}

}  // namespace vk::drivers
