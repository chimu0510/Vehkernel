// UART contract (ADR-0009).
//
// A platform's UART class satisfies Uart: 8 data bits, no parity, one stop
// bit, no flow control, and nothing that blocks. A Link does not take a UART
// type as a template argument: UartTransport binds any Uart at start-up, as
// I2cDevice binds an I2C bus.
#pragma once

#include <vk/hal/transport.hpp>

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::hal {

template <class T>
concept Uart = requires(T& uart, const T& const_uart, std::uint32_t baud, std::span<const std::uint8_t> out,
                        std::span<std::uint8_t> in) {
    // Sets the baud rate; false if the clock cannot reach it within 2 %.
    { uart.configure(baud) } -> std::same_as<bool>;
    // Bytes write() accepts now.
    { const_uart.write_space() } -> std::same_as<std::size_t>;
    // Queues what fits of `out` for transmission; returns the bytes taken.
    { uart.write(out) } -> std::same_as<std::size_t>;
    // Copies up to `in.size()` received bytes; returns the bytes copied.
    { uart.read(in) } -> std::same_as<std::size_t>;
};

// A Link transport on one UART.
class UartTransport final : public Transport {
public:
    template <Uart U>
    explicit UartTransport(U& uart)
        : uart_(&uart), write_space_(&write_space_on<U>), write_(&write_on<U>), read_(&read_on<U>)
    {
    }

    UartTransport(const UartTransport&) = delete;
    UartTransport& operator=(const UartTransport&) = delete;
    UartTransport(UartTransport&&) = delete;
    UartTransport& operator=(UartTransport&&) = delete;
    ~UartTransport() = default;

    [[nodiscard]] std::size_t write_space() const override { return write_space_(uart_); }
    std::size_t write(std::span<const std::uint8_t> data) override { return write_(uart_, data); }
    std::size_t read(std::span<std::uint8_t> out) override { return read_(uart_, out); }

private:
    using WriteSpaceFn = std::size_t (*)(const void*);
    using WriteFn = std::size_t (*)(void*, std::span<const std::uint8_t>);
    using ReadFn = std::size_t (*)(void*, std::span<std::uint8_t>);

    template <Uart U>
    static std::size_t write_space_on(const void* uart)
    {
        return static_cast<const U*>(uart)->write_space();
    }

    template <Uart U>
    static std::size_t write_on(void* uart, std::span<const std::uint8_t> data)
    {
        return static_cast<U*>(uart)->write(data);
    }

    template <Uart U>
    static std::size_t read_on(void* uart, std::span<std::uint8_t> out)
    {
        return static_cast<U*>(uart)->read(out);
    }

    void* uart_;
    WriteSpaceFn write_space_;
    WriteFn write_;
    ReadFn read_;
};

}  // namespace vk::hal
