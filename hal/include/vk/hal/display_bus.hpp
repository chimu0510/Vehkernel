// SPDX-License-Identifier: GPL-3.0-or-later
// Write-only display bus contract (ADR-0009, ADR-0022).
//
// A display controller such as the SSD1306 on a bus that only goes towards
// it: a reset line, and bytes sent as commands or as display data, which a
// data/command line tells apart. Nothing comes back, so a driver cannot tell
// whether a display is fitted. A platform's bus class, software SPI on GPIO
// lines or a simulated controller, satisfies DisplayBus; drivers talk to it
// through a DisplayDevice, which binds any DisplayBus at start-up.
#pragma once

#include <concepts>
#include <cstdint>
#include <span>

namespace vk::hal {

template <class T>
concept DisplayBus = requires(T& bus, bool active, std::span<const std::uint8_t> bytes) {
    // Drives the reset line: active holds the controller in reset.
    bus.set_reset(active);
    // Sends bytes with the data/command line at command.
    bus.write_commands(bytes);
    // Sends bytes with the data/command line at data.
    bus.write_data(bytes);
};

// One display on a display bus.
class DisplayDevice {
public:
    template <DisplayBus Bus>
    explicit DisplayDevice(Bus& bus)
        : bus_(&bus), set_reset_(&set_reset_on<Bus>), write_commands_(&write_commands_on<Bus>),
          write_data_(&write_data_on<Bus>)
    {
    }

    void set_reset(bool active) const { set_reset_(bus_, active); }
    void write_commands(std::span<const std::uint8_t> bytes) const { write_commands_(bus_, bytes); }
    void write_data(std::span<const std::uint8_t> bytes) const { write_data_(bus_, bytes); }

private:
    using SetResetFn = void (*)(void*, bool);
    using WriteFn = void (*)(void*, std::span<const std::uint8_t>);

    template <DisplayBus Bus>
    static void set_reset_on(void* bus, bool active)
    {
        static_cast<Bus*>(bus)->set_reset(active);
    }

    template <DisplayBus Bus>
    static void write_commands_on(void* bus, std::span<const std::uint8_t> bytes)
    {
        static_cast<Bus*>(bus)->write_commands(bytes);
    }

    template <DisplayBus Bus>
    static void write_data_on(void* bus, std::span<const std::uint8_t> bytes)
    {
        static_cast<Bus*>(bus)->write_data(bytes);
    }

    void* bus_;
    SetResetFn set_reset_;
    WriteFn write_commands_;
    WriteFn write_data_;
};

}  // namespace vk::hal
