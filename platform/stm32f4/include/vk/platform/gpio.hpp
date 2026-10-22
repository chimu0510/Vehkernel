// GPIO lines on STM32F4 (ADR-0009), through ST LL.
#pragma once

#include <vk/hal/gpio.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>

#include <cstdint>

namespace vk::platform {

namespace detail {

constexpr std::uint32_t gpio_port_stride = GPIOB_BASE - GPIOA_BASE;

inline GPIO_TypeDef* gpio_port(hal::Port port)
{
    // NOLINTNEXTLINE(performance-no-int-to-ptr): peripheral registers live at fixed addresses.
    return reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + gpio_port_stride * static_cast<std::uint32_t>(port));
}

inline void enable_gpio_clock(hal::Port port)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA << static_cast<std::uint32_t>(port));
}

template <hal::AfPinType Pin>
void configure_af_pin(std::uint32_t pull)
{
    enable_gpio_clock(Pin::port);
    GPIO_TypeDef* const port = gpio_port(Pin::port);
    constexpr std::uint32_t mask = 1U << Pin::number;
    if constexpr (Pin::number < 8) {
        LL_GPIO_SetAFPin_0_7(port, mask, Pin::af);
    } else {
        LL_GPIO_SetAFPin_8_15(port, mask, Pin::af);
    }
    LL_GPIO_SetPinOutputType(port, mask, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(port, mask, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(port, mask, pull);
    LL_GPIO_SetPinMode(port, mask, LL_GPIO_MODE_ALTERNATE);
}

}  // namespace detail

// An open-drain line on a GPIO_OUTPUT pin. Released, the external pull-up
// (with the internal one as a fallback) takes it high.
template <hal::GpioOutputPin Pin>
struct OpenDrain {
    static constexpr std::uint32_t mask = 1U << Pin::number;

    static void init()
    {
        detail::enable_gpio_clock(Pin::port);
        GPIO_TypeDef* const port = detail::gpio_port(Pin::port);
        LL_GPIO_SetOutputPin(port, mask);  // Released before it starts driving.
        LL_GPIO_SetPinOutputType(port, mask, LL_GPIO_OUTPUT_OPENDRAIN);
        LL_GPIO_SetPinSpeed(port, mask, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinPull(port, mask, LL_GPIO_PULL_UP);
        LL_GPIO_SetPinMode(port, mask, LL_GPIO_MODE_OUTPUT);
    }

    static void release() { LL_GPIO_SetOutputPin(detail::gpio_port(Pin::port), mask); }
    static void drive_low() { LL_GPIO_ResetOutputPin(detail::gpio_port(Pin::port), mask); }
    static bool is_high() { return LL_GPIO_IsInputPinSet(detail::gpio_port(Pin::port), mask) != 0U; }
};

// A push-pull line on a GPIO_OUTPUT pin, such as a software SPI signal.
template <hal::GpioOutputPin Pin>
struct PushPull {
    static constexpr std::uint32_t mask = 1U << Pin::number;

    // Starts driving the line at `high`.
    static void init(bool high)
    {
        detail::enable_gpio_clock(Pin::port);
        GPIO_TypeDef* const port = detail::gpio_port(Pin::port);
        write(high);  // At its level before it starts driving.
        LL_GPIO_SetPinOutputType(port, mask, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinSpeed(port, mask, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinPull(port, mask, LL_GPIO_PULL_NO);
        LL_GPIO_SetPinMode(port, mask, LL_GPIO_MODE_OUTPUT);
    }

    static void write(bool high)
    {
        if (high) {
            LL_GPIO_SetOutputPin(detail::gpio_port(Pin::port), mask);
        } else {
            LL_GPIO_ResetOutputPin(detail::gpio_port(Pin::port), mask);
        }
    }
};

// An input line on a GPIO_INPUT pin, with the pull resistor the board
// definition declares: none where the board drives it, as with revision-ID
// straps or a sense input, the internal pull-up for the C50C's user key.
template <hal::GpioInputPin Pin>
struct InputLine {
    static constexpr std::uint32_t mask = 1U << Pin::number;
    static constexpr std::uint32_t pull = Pin::pull == hal::Pull::Up     ? LL_GPIO_PULL_UP
                                          : Pin::pull == hal::Pull::Down ? LL_GPIO_PULL_DOWN
                                                                         : LL_GPIO_PULL_NO;

    static void init()
    {
        detail::enable_gpio_clock(Pin::port);
        GPIO_TypeDef* const port = detail::gpio_port(Pin::port);
        LL_GPIO_SetPinPull(port, mask, pull);
        LL_GPIO_SetPinMode(port, mask, LL_GPIO_MODE_INPUT);
    }

    static bool is_high() { return LL_GPIO_IsInputPinSet(detail::gpio_port(Pin::port), mask) != 0U; }
};

// A list of GPIO_INPUT pins read as a number, the first pin as the least
// significant bit, such as a board's revision-ID pins (ADR-0010).
template <class List>
struct InputBits;

template <hal::GpioInputPin... Pins>
struct InputBits<hal::PinList<Pins...>> {
    static void init() { (InputLine<Pins>::init(), ...); }

    static std::uint32_t read()
    {
        std::uint32_t value = 0;
        std::uint32_t bit = 0;
        ((value |= (InputLine<Pins>::is_high() ? 1U : 0U) << bit++), ...);
        return value;
    }
};

}  // namespace vk::platform
