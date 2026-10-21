// GPIO line contracts (ADR-0009), built on the generated typed pins.
//
// A platform drives a pin through a class template taking the pin's type;
// constraining that template with the role concepts below makes using a pin
// against its board role a compile error. The line concepts state what each
// platform's class must offer.
#pragma once

#include <vk/hal/pin.hpp>

#include <concepts>
#include <cstdint>
#include <type_traits>

namespace vk::hal {

namespace detail {
template <class T>
struct IsGpioOutput : std::false_type {};
template <Port P, std::uint8_t N>
struct IsGpioOutput<GpioOutput<P, N>> : std::true_type {};

template <class T>
struct IsGpioInput : std::false_type {};
template <Port P, std::uint8_t N, Pull R>
struct IsGpioInput<GpioInput<P, N, R>> : std::true_type {};
}  // namespace detail

// A pin whose board role is GPIO_OUTPUT.
template <class T>
concept GpioOutputPin = PinType<T> && detail::IsGpioOutput<T>::value;

// A pin whose board role is GPIO_INPUT.
template <class T>
concept GpioInputPin = PinType<T> && detail::IsGpioInput<T>::value;

// An open-drain line: released, it floats high through a pull-up; driven, it
// is pulled low. Reading returns the actual level, so a device holding the
// line low is seen.
template <class T>
concept OpenDrainLine = requires {
    T::init();
    T::release();
    T::drive_low();
    { T::is_high() } -> std::same_as<bool>;
};

}  // namespace vk::hal
