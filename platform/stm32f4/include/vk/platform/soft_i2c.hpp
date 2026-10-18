// Software I2C on two GPIO_OUTPUT pins (ADR-0015).
//
// The CPU busy-waits through every bit, so one transfer blocks its work
// queue for its whole duration; ADR-0015 accounts for that cost. Both lines
// are open-drain, so a level shifter or device may hold them low, and SCL
// held low by a device (clock stretching) is waited for, up to a timeout.
#pragma once

#include <vk/hal/gpio.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/platform/cycle_counter.hpp>
#include <vk/platform/gpio.hpp>

#include <board.hpp>

#include <cstdint>
#include <span>

namespace vk::platform {

template <hal::GpioOutputPin SclPin, hal::GpioOutputPin SdaPin, std::uint32_t BusHz = 400'000>
class SoftI2c {
public:
    static_assert(BusHz > 0 && BusHz <= 400'000, "software I2C runs at fast mode (400 kHz) or below");

    // Configures both lines and frees a bus a device may still hold from an
    // interrupted transfer.
    void init()
    {
        Scl::init();
        Sda::init();
        recover();
    }

    bool write(std::uint8_t address, std::span<const std::uint8_t> data)
    {
        bool ok = start() && send_byte(write_address(address));
        for (std::size_t i = 0; ok && i < data.size(); ++i) {
            ok = send_byte(data[i]);
        }
        stop();
        return ok;
    }

    bool read(std::uint8_t address, std::span<std::uint8_t> data)
    {
        const bool ok = start() && send_byte(read_address(address)) && receive(data);
        stop();
        return ok;
    }

    bool read_register(std::uint8_t address, std::uint8_t reg, std::span<std::uint8_t> data)
    {
        const bool ok = start() && send_byte(write_address(address)) && send_byte(reg) && start() &&
                        send_byte(read_address(address)) && receive(data);
        stop();
        return ok;
    }

private:
    using Scl = OpenDrain<SclPin>;
    using Sda = OpenDrain<SdaPin>;

    static constexpr std::uint32_t half_period_cycles = vk::board::clock::sysclk_hz / BusHz / 2;
    // A device may stretch the clock for this long before the transfer fails.
    static constexpr std::uint32_t stretch_timeout_cycles = vk::board::clock::sysclk_hz / 1000;  // 1 ms

    static_assert(hal::OpenDrainLine<Scl> && hal::OpenDrainLine<Sda>);

    static constexpr std::uint8_t write_address(std::uint8_t address) { return static_cast<std::uint8_t>(address << 1U); }
    static constexpr std::uint8_t read_address(std::uint8_t address)
    {
        return static_cast<std::uint8_t>((address << 1U) | 1U);
    }

    static void half_period() { delay_cycles(half_period_cycles); }

    // Releases SCL and waits for it to go high.
    static bool raise_scl()
    {
        Scl::release();
        const std::uint32_t start = cycles();
        while (!Scl::is_high()) {
            if (cycles() - start > stretch_timeout_cycles) {
                return false;
            }
        }
        return true;
    }

    // START, or a repeated START when SCL is low mid-transfer.
    static bool start()
    {
        Sda::release();
        half_period();
        if (!raise_scl()) {
            return false;
        }
        half_period();
        Sda::drive_low();
        half_period();
        Scl::drive_low();
        return true;
    }

    static void stop()
    {
        Sda::drive_low();
        half_period();
        static_cast<void>(raise_scl());
        half_period();
        Sda::release();
        half_period();
    }

    // Sends one bit with SCL low on entry and exit.
    static bool send_bit(bool bit)
    {
        if (bit) {
            Sda::release();
        } else {
            Sda::drive_low();
        }
        half_period();
        if (!raise_scl()) {
            return false;
        }
        half_period();
        Scl::drive_low();
        return true;
    }

    static bool receive_bit(bool& bit)
    {
        Sda::release();
        half_period();
        if (!raise_scl()) {
            return false;
        }
        half_period();
        bit = Sda::is_high();
        Scl::drive_low();
        return true;
    }

    // Sends a byte; true when the device acknowledged it.
    static bool send_byte(std::uint8_t byte)
    {
        for (int i = 7; i >= 0; --i) {
            if (!send_bit(((byte >> i) & 1U) != 0U)) {
                return false;
            }
        }
        bool nack = true;
        return receive_bit(nack) && !nack;
    }

    // Receives the bytes, acknowledging all but the last.
    static bool receive(std::span<std::uint8_t> data)
    {
        for (std::size_t n = 0; n < data.size(); ++n) {
            std::uint8_t byte = 0;
            for (int i = 0; i < 8; ++i) {
                bool bit = false;
                if (!receive_bit(bit)) {
                    return false;
                }
                byte = static_cast<std::uint8_t>((byte << 1U) | (bit ? 1U : 0U));
            }
            data[n] = byte;
            if (!send_bit(n + 1 == data.size())) {  // ACK is low, NACK on the last byte.
                return false;
            }
        }
        return true;
    }

    // Clocks out a device stuck mid-byte holding SDA low, then sends STOP.
    static void recover()
    {
        Sda::release();
        for (int i = 0; i < 9 && !Sda::is_high(); ++i) {
            Scl::drive_low();
            half_period();
            static_cast<void>(raise_scl());
            half_period();
        }
        Scl::drive_low();
        half_period();
        stop();
    }
};

}  // namespace vk::platform
