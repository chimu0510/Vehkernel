// USART baud rate register values for STM32F4, at 16x oversampling.
//
// Free of hardware access, so it is unit-tested on the development host.
#pragma once

#include <cstdint>

namespace vk::platform {

// BRR for `baud` from the peripheral clock: USARTDIV = pclk / (16 * baud),
// 12 bits of mantissa and 4 of fraction, rounded to the nearest step.
constexpr std::uint32_t usart_brr(std::uint32_t pclk_hz, std::uint32_t baud)
{
    return (pclk_hz + baud / 2) / baud;
}

// The baud rate a BRR value gives.
constexpr std::uint32_t usart_baud(std::uint32_t pclk_hz, std::uint32_t brr)
{
    return brr == 0 ? 0 : (pclk_hz + brr / 2) / brr;
}

// Whether `baud` is reachable within 2 %, the most a UART receiver tolerates
// with margin.
constexpr bool usart_baud_reachable(std::uint32_t pclk_hz, std::uint32_t baud)
{
    if (baud == 0 || pclk_hz / baud < 16) {
        return false;
    }
    const std::uint32_t actual = usart_baud(pclk_hz, usart_brr(pclk_hz, baud));
    const std::uint32_t error = actual > baud ? actual - baud : baud - actual;
    return error * 50U <= baud;
}

}  // namespace vk::platform
