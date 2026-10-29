// SPDX-License-Identifier: GPL-3.0-or-later
// Register model of the TDK InvenSense ICM20948 (datasheet DS-000189).
//
// One model serves as the fake device of the driver's unit tests and as the
// simulated IMU in SITL (ADR-0018). It implements the registers the driver
// uses: bank selection, WHO_AM_I, power management, reset, sample rate and
// full-scale configuration, and the accelerometer, gyroscope and temperature
// outputs, which follow the values set on the model scaled by the configured
// full-scale range. It also models the I2C master (slaves 0 and 4), the FIFO
// and the internal Asahi Kasei AK09916 magnetometer: WHO_AM_I, continuous
// mode 4, ST1, ST2 and overflow, with master cycles at the output data rate.
#pragma once

#include <vk/sim/i2c_bus.hpp>

#include <array>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace vk::sim {

class Icm20948 final : public RegisterDevice {
public:
    static constexpr std::uint8_t default_address = 0x68;

    using Clock = std::uint64_t (*)() noexcept;

    explicit Icm20948(Clock clock);

    // Outputs, in the sensor's own frame (x, y, z up): specific force in
    // m/s^2 (lying flat reads +9.81 on z), angular rate in rad/s.
    void set_accel(const std::array<float, 3>& m_s2) { accel_ = m_s2; }
    void set_gyro(const std::array<float, 3>& rad_s) { gyro_ = rad_s; }
    void set_temperature(float celsius) { temperature_ = celsius; }

    // Magnetometer field in the sensor's own frame (x forward, y left, z up) in µT.
    void set_field(const std::array<float, 3>& uT) { field_ = uT; }

    // Adds uniform noise of the given amplitude to every accelerometer and
    // gyroscope reading, as a real sensor shows; off by default.
    void set_noise(float accel_m_s2, float gyro_rad_s)
    {
        accel_noise_ = accel_m_s2;
        gyro_noise_ = gyro_rad_s;
    }

    // Adds uniform noise of the given amplitude to every magnetometer reading in µT.
    void set_field_noise(float uT) { mag_noise_ = uT; }

    // Simulates a magnetic sensor overflow condition (HOFL bit in ST2).
    void set_mag_overflow(bool overflow) { mag_overflow_ = overflow; }

    // A magnetometer that stops responding does not answer through the I2C master.
    void set_mag_responding(bool responding) { mag_responding_ = responding; }

    // Pauses magnetometer sample generation without changing mode, for tests.
    void set_mag_paused(bool paused) noexcept { mag_paused_ = paused; }

    // Test hooks to push stray bytes into the FIFO for testing bad counts and overflow.
    void push_fifo_byte(std::uint8_t byte);
    void push_fifo(std::span<const std::uint8_t> bytes);

    struct ReadTransfer {
        std::uint8_t reg = 0;
        std::size_t length = 0;
        std::uint64_t count = 0;
    };

    // Read transfers counted by starting register and length.
    [[nodiscard]] std::vector<ReadTransfer> read_transfers() const;

    // Sets the outputs just before each sample is taken, for example from a
    // motion profile; the values set on the model are used otherwise.
    class OutputSource {
    public:
        OutputSource() = default;
        OutputSource(const OutputSource&) = delete;
        OutputSource& operator=(const OutputSource&) = delete;
        OutputSource(OutputSource&&) = delete;
        OutputSource& operator=(OutputSource&&) = delete;

        virtual void update(Icm20948& imu) = 0;
        virtual std::optional<std::array<float, 3>> field_at(std::uint64_t time_us)
        {
            static_cast<void>(time_us);
            return std::nullopt;
        }

    protected:
        ~OutputSource() = default;
    };

    void set_output_source(OutputSource* source) { source_ = source; }

    // A device that stops responding fails every transfer, as with a loose cable.
    void set_responding(bool responding) { responding_ = responding; }

    // Register contents, for tests.
    [[nodiscard]] std::uint8_t register_value(std::uint8_t bank, std::uint8_t reg) const;
    [[nodiscard]] std::uint8_t ak09916_register_value(std::uint8_t reg) const;
    [[nodiscard]] bool sleeping() const;
    [[nodiscard]] int resets() const { return resets_; }

    bool write(std::span<const std::uint8_t> data) override;
    bool read(std::span<std::uint8_t> data) override;

protected:
    void write_register(std::uint8_t reg, std::uint8_t value) override;
    std::uint8_t read_register(std::uint8_t reg) override;

private:
    void reset();
    [[nodiscard]] std::uint8_t bank() const;
    // Samples the outputs once per read transfer, so a burst read is consistent.
    void latch_outputs();
    std::uint8_t output_register(std::uint8_t reg);
    float noise(float amplitude);
    [[nodiscard]] std::uint64_t now_us() const;
    void advance_master_to(std::uint64_t now);
    void run_master_cycle(std::uint64_t cycle_time);

    std::array<std::array<std::uint8_t, 128>, 4> banks_{};
    std::uint8_t bank_sel_ = 0;
    std::array<float, 3> accel_{0.0F, 0.0F, 9.80665F};
    std::array<float, 3> gyro_{};
    float temperature_ = 25.0F;
    float accel_noise_ = 0.0F;
    float gyro_noise_ = 0.0F;
    std::uint32_t noise_state_ = 0x12345678;
    std::array<std::int16_t, 7> outputs_{};  // ACCEL X..Z, GYRO X..Z, TEMP
    OutputSource* source_ = nullptr;
    bool responding_ = true;
    int resets_ = 0;

    // Magnetometer (AK09916), I2C master and FIFO state:
    Clock clock_ = nullptr;
    std::array<float, 3> field_{0.0F, 0.0F, 0.0F};
    float mag_noise_ = 0.0F;
    bool mag_overflow_ = false;
    bool mag_responding_ = true;
    bool mag_data_ready_ = false;
    std::uint64_t mag_last_sample_us_ = 0;
    std::uint8_t ak_cntl2_ = 0;
    std::array<std::uint8_t, 6> current_mag_raw_{};
    std::array<std::uint8_t, 9> mag_bytes_{};  // ST1, HXL..HZH, reserved, ST2

    std::uint64_t last_master_cycle_us_ = 0;
    std::uint64_t master_sub_us_ = 0;
    std::uint8_t i2c_mst_status_ = 0;
    bool slv4_pending_ = false;
    std::deque<std::uint8_t> fifo_{};
    bool mag_paused_ = false;
    std::map<std::pair<std::uint8_t, std::size_t>, std::uint64_t> read_counts_{};
};

}  // namespace vk::sim
