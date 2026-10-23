// SPDX-License-Identifier: GPL-3.0-or-later
// I2C bus contract (ADR-0009).
//
// A platform's I2C bus class, hardware, software or simulated, satisfies
// I2cBus. Drivers are not templates: they talk to one device through an
// I2cDevice, which binds any I2cBus and a 7-bit address at start-up.
#pragma once

#include <array>
#include <concepts>
#include <cstdint>
#include <span>

namespace vk::hal {

// Every transfer returns false when the device does not acknowledge or the
// bus fails; the bus is left idle either way.
template <class T>
concept I2cBus = requires(T& bus, std::uint8_t address, std::uint8_t reg, std::span<const std::uint8_t> out,
                          std::span<std::uint8_t> in) {
    // START, address + write, the bytes, STOP.
    { bus.write(address, out) } -> std::same_as<bool>;
    // START, address + read, the bytes, STOP.
    { bus.read(address, in) } -> std::same_as<bool>;
    // START, address + write, `reg`, repeated START, address + read, the bytes, STOP.
    { bus.read_register(address, reg, in) } -> std::same_as<bool>;
};

// One device on an I2C bus.
class I2cDevice {
public:
    template <I2cBus Bus>
    I2cDevice(Bus& bus, std::uint8_t address)
        : bus_(&bus), address_(address), write_(&write_on<Bus>), read_(&read_on<Bus>),
          read_register_(&read_register_on<Bus>)
    {
    }

    [[nodiscard]] std::uint8_t address() const { return address_; }

    [[nodiscard]] bool write(std::span<const std::uint8_t> data) const { return write_(bus_, address_, data); }
    [[nodiscard]] bool read(std::span<std::uint8_t> data) const { return read_(bus_, address_, data); }

    [[nodiscard]] bool read_register(std::uint8_t reg, std::span<std::uint8_t> data) const
    {
        return read_register_(bus_, address_, reg, data);
    }

    [[nodiscard]] bool write_register(std::uint8_t reg, std::uint8_t value) const
    {
        const std::array<std::uint8_t, 2> data{reg, value};
        return write(data);
    }

private:
    using WriteFn = bool (*)(void*, std::uint8_t, std::span<const std::uint8_t>);
    using ReadFn = bool (*)(void*, std::uint8_t, std::span<std::uint8_t>);
    using ReadRegisterFn = bool (*)(void*, std::uint8_t, std::uint8_t, std::span<std::uint8_t>);

    template <I2cBus Bus>
    static bool write_on(void* bus, std::uint8_t address, std::span<const std::uint8_t> data)
    {
        return static_cast<Bus*>(bus)->write(address, data);
    }

    template <I2cBus Bus>
    static bool read_on(void* bus, std::uint8_t address, std::span<std::uint8_t> data)
    {
        return static_cast<Bus*>(bus)->read(address, data);
    }

    template <I2cBus Bus>
    static bool read_register_on(void* bus, std::uint8_t address, std::uint8_t reg, std::span<std::uint8_t> data)
    {
        return static_cast<Bus*>(bus)->read_register(address, reg, data);
    }

    void* bus_;
    std::uint8_t address_;
    WriteFn write_;
    ReadFn read_;
    ReadRegisterFn read_register_;
};

}  // namespace vk::hal
