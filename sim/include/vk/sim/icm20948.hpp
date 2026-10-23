// SPDX-License-Identifier: GPL-3.0-or-later
// Register model of the TDK InvenSense ICM20948 (datasheet DS-000189).
//
// One model serves as the fake device of the driver's unit tests and as the
// simulated IMU in SITL (ADR-0018). It implements the registers the driver
// uses: bank selection, WHO_AM_I, power management, reset, sample rate and
// full-scale configuration, and the accelerometer, gyroscope and temperature
// outputs, which follow the values set on the model scaled by the configured
// full-scale range. The magnetometer is not modelled.
#pragma once

#include <vk/sim/i2c_bus.hpp>

#include <array>
#include <cstdint>

namespace vk::sim {

class Icm20948 final : public RegisterDevice {
public:
    static constexpr std::uint8_t default_address = 0x68;

    // Outputs, in the sensor's own frame (x, y, z up): specific force in
    // m/s^2 (lying flat reads +9.81 on z), angular rate in rad/s.
    void set_accel(const std::array<float, 3>& m_s2) { accel_ = m_s2; }
    void set_gyro(const std::array<float, 3>& rad_s) { gyro_ = rad_s; }
    void set_temperature(float celsius) { temperature_ = celsius; }

    // Adds uniform noise of the given amplitude to every accelerometer and
    // gyroscope reading, as a real sensor shows; off by default.
    void set_noise(float accel_m_s2, float gyro_rad_s)
    {
        accel_noise_ = accel_m_s2;
        gyro_noise_ = gyro_rad_s;
    }

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

    protected:
        ~OutputSource() = default;
    };

    void set_output_source(OutputSource* source) { source_ = source; }

    // A device that stops responding fails every transfer, as with a loose cable.
    void set_responding(bool responding) { responding_ = responding; }

    // Register contents, for tests.
    [[nodiscard]] std::uint8_t register_value(std::uint8_t bank, std::uint8_t reg) const;
    [[nodiscard]] bool sleeping() const;
    [[nodiscard]] int resets() const { return resets_; }

    bool write(std::span<const std::uint8_t> data) override;
    bool read(std::span<std::uint8_t> data) override;

    Icm20948();

protected:
    void write_register(std::uint8_t reg, std::uint8_t value) override;
    std::uint8_t read_register(std::uint8_t reg) override;

private:
    void reset();
    [[nodiscard]] std::uint8_t bank() const;
    // Samples the outputs once per read transfer, so a burst read is consistent.
    void latch_outputs();
    [[nodiscard]] std::uint8_t output_register(std::uint8_t reg) const;
    float noise(float amplitude);

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
};

}  // namespace vk::sim
