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

}  // namespace vk::platform
