// SPDX-License-Identifier: GPL-3.0-or-later
// Actuator outputs on STM32F4 (ADR-0009, spec 0006): an H-bridge driven by a
// PWM channel of an advanced timer (TIM1 or TIM8) and two GPIO direction
// lines, and a quadrature encoder on CH1 and CH2 of a general-purpose timer.
//
// One class per board output, from the board definition's typed actuator
// constants. Outputs on one PWM timer share its frequency. Every duty
// refreshes the output watchdog, whose interrupt calls stop() on a stall.
#pragma once

#include <vk/hal/actuator.hpp>
#include <vk/platform/gpio.hpp>
#include <vk/platform/pwm_timing.hpp>

#include <board.hpp>

#include <stm32f4xx.h>
#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_gpio.h>
#include <stm32f4xx_ll_tim.h>

#include <algorithm>
#include <cstdint>

namespace vk::platform {

// Refreshes the output watchdog; defined with its interrupt.
void refresh_output_watchdog();

namespace detail {

inline TIM_TypeDef* timer(unsigned number)
{
    switch (number) {
    case 1:
        return TIM1;
    case 2:
        return TIM2;
    case 3:
        return TIM3;
    case 4:
        return TIM4;
    case 5:
        return TIM5;
    default:
        return TIM8;
    }
}

inline void enable_timer_clock(unsigned number)
{
    switch (number) {
    case 1:
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
        break;
    case 2:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
        break;
    case 3:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
        break;
    case 4:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
        break;
    case 5:
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
        break;
    default:
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
        break;
    }
}

constexpr std::uint32_t ll_channel(unsigned channel)
{
    switch (channel) {
    case 1:
        return LL_TIM_CHANNEL_CH1;
    case 2:
        return LL_TIM_CHANNEL_CH2;
    case 3:
        return LL_TIM_CHANNEL_CH3;
    default:
        return LL_TIM_CHANNEL_CH4;
    }
}

template <class Pin>
void configure_output_low()
{
    enable_gpio_clock(Pin::port);
    GPIO_TypeDef* const port = gpio_port(Pin::port);
    constexpr std::uint32_t mask = 1U << Pin::number;
    LL_GPIO_ResetOutputPin(port, mask);  // Low before it starts driving.
    LL_GPIO_SetPinOutputType(port, mask, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinPull(port, mask, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(port, mask, LL_GPIO_MODE_OUTPUT);
}

template <class Pin>
void write(bool high)
{
    constexpr std::uint32_t mask = 1U << Pin::number;
    if (high) {
        LL_GPIO_SetOutputPin(gpio_port(Pin::port), mask);
    } else {
        LL_GPIO_ResetOutputPin(gpio_port(Pin::port), mask);
    }
}

}  // namespace detail

template <class Output>
class MotorOutputOn final : public hal::PwmOutput, public hal::DirectionLines, public hal::QuadratureEncoder {
public:
    static_assert(Output::pwm_timer == 1 || Output::pwm_timer == 8, "motor PWM needs an advanced timer, TIM1 or TIM8");
    static_assert(Output::encoder_timer >= 2 && Output::encoder_timer <= 5, "encoders run on TIM2 to TIM5");

    // Pins and the encoder; the PWM timer starts with set_frequency(). The
    // PWM pin stays a low GPIO output until then.
    void init()
    {
        detail::configure_output_low<typename Output::in1>();
        detail::configure_output_low<typename Output::in2>();
        detail::configure_af_pin<typename Output::encoder_a>(LL_GPIO_PULL_UP);
        detail::configure_af_pin<typename Output::encoder_b>(LL_GPIO_PULL_UP);
        detail::enable_timer_clock(Output::encoder_timer);
        TIM_TypeDef* const encoder = detail::timer(Output::encoder_timer);
        LL_TIM_IC_SetActiveInput(encoder, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
        LL_TIM_IC_SetActiveInput(encoder, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
        LL_TIM_IC_SetFilter(encoder, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1_N4);
        LL_TIM_IC_SetFilter(encoder, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1_N4);
        LL_TIM_SetEncoderMode(encoder, LL_TIM_ENCODERMODE_X4_TI12);
        LL_TIM_SetAutoReload(encoder, 0xFFFFU);  // 16 bits on every timer width
        LL_TIM_EnableCounter(encoder);
    }

    bool set_frequency(std::uint32_t hz) override
    {
        constexpr std::uint32_t clock = Output::pwm_timer == 1 || Output::pwm_timer == 8
                                            ? vk::board::clock::apb2_timer_hz
                                            : vk::board::clock::apb1_timer_hz;
        auto timing = pwm_timing(clock, hz);
        const bool reached = timing.has_value();
        if (!reached) {
            timing = pwm_timing(clock, static_cast<std::uint32_t>(vk::board::defaults::MOT_PWM_FREQ));
        }
        if (!timing) {
            return false;
        }
        TIM_TypeDef* const pwm = detail::timer(Output::pwm_timer);
        detail::enable_timer_clock(Output::pwm_timer);
        // A breakpoint freezes the timer, which clears MOE: with OSSI and the
        // idle levels low, the outputs then drive low (ADR-0005).
        if constexpr (Output::pwm_timer == 8) {
            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM8_STOP;
        } else {
            DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
        }
        pwm->CR2 = (pwm->CR2 & ~advanced_timer_ois_mask) | advanced_timer_ois;
        LL_TIM_SetPrescaler(pwm, timing->prescaler);
        LL_TIM_SetAutoReload(pwm, timing->period);
        LL_TIM_EnableARRPreload(pwm);
        constexpr std::uint32_t channel = detail::ll_channel(Output::pwm_channel);
        LL_TIM_OC_SetMode(pwm, channel, LL_TIM_OCMODE_PWM1);
        LL_TIM_OC_EnablePreload(pwm, channel);
        write_compare(0);
        LL_TIM_CC_EnableChannel(pwm, channel);
        pwm->BDTR = advanced_timer_bdtr;
        LL_TIM_GenerateEvent_UPDATE(pwm);
        LL_TIM_EnableCounter(pwm);
        detail::configure_af_pin<typename Output::pwm>(LL_GPIO_PULL_DOWN);
        return reached;
    }

    void set_duty(float duty) override
    {
        refresh_output_watchdog();
        TIM_TypeDef* const pwm = detail::timer(Output::pwm_timer);
        const auto ticks = static_cast<std::uint32_t>(std::clamp(duty, 0.0F, 1.0F) *
                                                      static_cast<float>(LL_TIM_GetAutoReload(pwm) + 1U));
        write_compare(ticks);
    }

    void set(bool in1, bool in2) override
    {
        detail::write<typename Output::in1>(in1);
        detail::write<typename Output::in2>(in2);
    }

    [[nodiscard]] std::uint16_t count() const override
    {
        return static_cast<std::uint16_t>(LL_TIM_GetCounter(detail::timer(Output::encoder_timer)));
    }

    // The output watchdog's stop: no duty, both lines low; safe in an interrupt.
    static void stop()
    {
        write_compare(0);
        detail::write<typename Output::in1>(false);
        detail::write<typename Output::in2>(false);
    }

private:
    static void write_compare(std::uint32_t ticks)
    {
        TIM_TypeDef* const pwm = detail::timer(Output::pwm_timer);
        switch (Output::pwm_channel) {
        case 1:
            LL_TIM_OC_SetCompareCH1(pwm, ticks);
            break;
        case 2:
            LL_TIM_OC_SetCompareCH2(pwm, ticks);
            break;
        case 3:
            LL_TIM_OC_SetCompareCH3(pwm, ticks);
            break;
        default:
            LL_TIM_OC_SetCompareCH4(pwm, ticks);
            break;
        }
    }
};

}  // namespace vk::platform
