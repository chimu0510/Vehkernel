// The I2C contract (ADR-0009): an I2cDevice forwards to whichever I2cBus it
// was bound to, with its own address.
#include <vk/hal/gpio.hpp>
#include <vk/hal/i2c.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace {

using vk::hal::Port;

static_assert(vk::hal::GpioOutputPin<vk::hal::GpioOutput<Port::B, 9>>);
static_assert(!vk::hal::GpioOutputPin<vk::hal::GpioInput<Port::B, 9>>);
static_assert(!vk::hal::GpioOutputPin<vk::hal::AfPin<Port::B, 9, 4, "I2C1_SDA">>);
static_assert(vk::hal::GpioInputPin<vk::hal::GpioInput<Port::D, 3>>);

struct RecordingBus {
    std::uint8_t last_address = 0;
    std::uint8_t last_register = 0;
    std::vector<std::uint8_t> written;
    bool acknowledge = true;

    bool write(std::uint8_t address, std::span<const std::uint8_t> data)
    {
        last_address = address;
        written.assign(data.begin(), data.end());
        return acknowledge;
    }

    bool read(std::uint8_t address, std::span<std::uint8_t> data)
    {
        last_address = address;
        for (auto& byte : data) {
            byte = 0x5A;
        }
        return acknowledge;
    }

    bool read_register(std::uint8_t address, std::uint8_t reg, std::span<std::uint8_t> data)
    {
        last_register = reg;
        return read(address, data);
    }
};

static_assert(vk::hal::I2cBus<RecordingBus>);

TEST(I2cDevice, ForwardsTransfersWithItsAddress)
{
    RecordingBus bus;
    const vk::hal::I2cDevice device(bus, 0x68);
    std::array<std::uint8_t, 2> in{};
    ASSERT_TRUE(device.read_register(0x2D, in));
    EXPECT_EQ(bus.last_address, 0x68);
    EXPECT_EQ(bus.last_register, 0x2D);
    EXPECT_EQ(in[1], 0x5A);
}

TEST(I2cDevice, WriteRegisterSendsRegisterThenValue)
{
    RecordingBus bus;
    const vk::hal::I2cDevice device(bus, 0x0C);
    ASSERT_TRUE(device.write_register(0x7F, 0x20));
    EXPECT_EQ(bus.written, (std::vector<std::uint8_t>{0x7F, 0x20}));
}

TEST(I2cDevice, ReportsAMissingAcknowledge)
{
    RecordingBus bus;
    bus.acknowledge = false;
    const vk::hal::I2cDevice device(bus, 0x68);
    std::array<std::uint8_t, 1> in{};
    EXPECT_FALSE(device.read(in));
}

}  // namespace
