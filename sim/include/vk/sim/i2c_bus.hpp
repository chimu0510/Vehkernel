// SPDX-License-Identifier: GPL-3.0-or-later
// Simulated I2C peripheral bus (ADR-0018).
//
// SITL's I2C bus, and the fake peripheral bus of unit tests: register models
// of devices attach at their addresses, and the real drivers talk to them
// through the same I2C contract as on hardware.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::sim {

// A device on the simulated bus. Each call is one transfer between START
// and STOP; returning false is a missing acknowledge.
class I2cDeviceModel {
public:
    I2cDeviceModel() = default;
    virtual ~I2cDeviceModel() = default;
    I2cDeviceModel(const I2cDeviceModel&) = delete;
    I2cDeviceModel& operator=(const I2cDeviceModel&) = delete;
    I2cDeviceModel(I2cDeviceModel&&) = delete;
    I2cDeviceModel& operator=(I2cDeviceModel&&) = delete;

    virtual bool write(std::span<const std::uint8_t> data) = 0;
    virtual bool read(std::span<std::uint8_t> data) = 0;
};

class I2cBus {
public:
    static constexpr std::size_t max_devices = 8;

    // Attaches `device` at a 7-bit address, replacing any device there.
    void attach(std::uint8_t address, I2cDeviceModel& device);
    void detach(std::uint8_t address);

    bool write(std::uint8_t address, std::span<const std::uint8_t> data);
    bool read(std::uint8_t address, std::span<std::uint8_t> data);
    bool read_register(std::uint8_t address, std::uint8_t reg, std::span<std::uint8_t> data);

private:
    struct Slot {
        std::uint8_t address = 0;
        I2cDeviceModel* device = nullptr;
    };

    I2cDeviceModel* find(std::uint8_t address);

    std::array<Slot, max_devices> slots_{};
};

// Common shape of register-mapped I2C devices: the first byte written sets
// the register pointer, further bytes write registers, reads return
// registers; the pointer advances after each byte.
class RegisterDevice : public I2cDeviceModel {
public:
    bool write(std::span<const std::uint8_t> data) override;
    bool read(std::span<std::uint8_t> data) override;

protected:
    virtual void write_register(std::uint8_t reg, std::uint8_t value) = 0;
    virtual std::uint8_t read_register(std::uint8_t reg) = 0;
    [[nodiscard]] std::uint8_t pointer() const { return pointer_; }

private:
    std::uint8_t pointer_ = 0;
};

}  // namespace vk::sim
