// The UART contract, its Link transport binding, and the STM32F4 baud rate
// arithmetic (ADR-0009).
#include <vk/hal/uart.hpp>
#include <vk/platform/usart_baud.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace {

// Takes up to `space` bytes per write and hands back what it was given to read.
class FakeUart {
public:
    bool configure(std::uint32_t baud)
    {
        configured_baud = baud;
        return true;
    }

    [[nodiscard]] std::size_t write_space() const { return space; }

    std::size_t write(std::span<const std::uint8_t> data)
    {
        const std::size_t size = std::min(space, data.size());
        written.insert(written.end(), data.begin(), data.begin() + static_cast<std::ptrdiff_t>(size));
        return size;
    }

    std::size_t read(std::span<std::uint8_t> out)
    {
        const std::size_t size = std::min(out.size(), received.size());
        std::copy_n(received.begin(), size, out.begin());
        received.erase(received.begin(), received.begin() + static_cast<std::ptrdiff_t>(size));
        return size;
    }

    std::uint32_t configured_baud = 0;
    std::size_t space = 3;
    std::vector<std::uint8_t> written;
    std::vector<std::uint8_t> received;
};

static_assert(vk::hal::Uart<FakeUart>);

TEST(UartTransport, ForwardsToTheBoundUart)
{
    FakeUart uart;
    vk::hal::UartTransport transport(uart, 115200);
    vk::hal::Transport& link_side = transport;

    EXPECT_EQ(link_side.write_space(), 3U);
    const std::array<std::uint8_t, 5> data{1, 2, 3, 4, 5};
    EXPECT_EQ(link_side.write(data), 3U);  // what fits
    EXPECT_EQ(uart.written, (std::vector<std::uint8_t>{1, 2, 3}));

    uart.received = {7, 8};
    std::array<std::uint8_t, 4> in{};
    EXPECT_EQ(link_side.read(in), 2U);
    EXPECT_EQ(in[0], 7);
    EXPECT_EQ(in[1], 8);
    EXPECT_EQ(link_side.read(in), 0U);
}

TEST(UsartBaud, TheLinkRateIsReachableFromTheC50cApb1Clock)
{
    // USART3 on APB1 at 42 MHz: BRR 46 gives 913043 baud, 0.9 % slow.
    EXPECT_EQ(vk::platform::usart_brr(42'000'000, 921'600), 46U);
    EXPECT_EQ(vk::platform::usart_baud(42'000'000, 46), 913'043U);
    EXPECT_TRUE(vk::platform::usart_baud_reachable(42'000'000, 921'600));
    EXPECT_TRUE(vk::platform::usart_baud_reachable(84'000'000, 115'200));
}

TEST(UsartBaud, RatesTooFastOrTooFarOffAreRefused)
{
    EXPECT_FALSE(vk::platform::usart_baud_reachable(42'000'000, 3'000'000));  // under 16 clocks per bit
    EXPECT_FALSE(vk::platform::usart_baud_reachable(42'000'000, 2'550'000));  // BRR 16 gives 2.625 Mbaud, 2.9 % off
    EXPECT_FALSE(vk::platform::usart_baud_reachable(42'000'000, 0));
}

}  // namespace
