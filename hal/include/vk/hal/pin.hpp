// Typed pin descriptions emitted by the board generator (ADR-0009, ADR-0011).
//
// Each board role becomes its own type, so passing a pin to code expecting a
// different function, or a GPIO where an alternate function is needed, fails
// to compile.
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

namespace vk::hal {

enum class Port : std::uint8_t { A, B, C, D, E, F, G, H, I, J, K };

// A peripheral signal name such as "USART1_TX", usable as a template argument.
template <std::size_t N>
struct Signal {
    std::array<char, N> text{};

    // Implicit and taking a C array on purpose: the argument is a string literal.
    constexpr Signal(const char (&name)[N])  // NOLINT(google-explicit-constructor,modernize-avoid-c-arrays)
    {
        for (std::size_t i = 0; i < N; ++i) {
            text[i] = name[i];
        }
    }

    [[nodiscard]] constexpr std::string_view view() const { return {text.data(), N - 1}; }
};

template <Port P, std::uint8_t Number>
struct PinId {
    static constexpr Port port = P;
    static constexpr std::uint8_t number = Number;
};

// A pin routed to a peripheral through alternate function Af.
template <Port P, std::uint8_t Number, std::uint8_t Af, Signal S>
struct AfPin : PinId<P, Number> {
    static constexpr std::uint8_t af = Af;
    static constexpr std::string_view signal = S.view();
};

// A pin used as an analog input or output, such as an ADC channel.
template <Port P, std::uint8_t Number, Signal S>
struct AnalogPin : PinId<P, Number> {
    static constexpr std::string_view signal = S.view();
};

template <Port P, std::uint8_t Number>
struct GpioInput : PinId<P, Number> {};

template <Port P, std::uint8_t Number>
struct GpioOutput : PinId<P, Number> {};

template <class T>
concept PinType = requires {
    { T::port } -> std::convertible_to<Port>;
    { T::number } -> std::convertible_to<std::uint8_t>;
};

template <class T>
concept AfPinType = PinType<T> && requires {
    { T::af } -> std::convertible_to<std::uint8_t>;
    { T::signal } -> std::convertible_to<std::string_view>;
};

template <PinType... Pins>
struct PinList {
    static constexpr std::size_t size = sizeof...(Pins);
};

namespace detail {
template <PinType A, PinType B>
constexpr bool same_pin = A::port == B::port && A::number == B::number;

template <PinType First, PinType... Rest>
constexpr bool distinct_from_rest = (!same_pin<First, Rest> && ...);

template <class List>
struct Distinct;

template <>
struct Distinct<PinList<>> : std::true_type {};

template <PinType First, PinType... Rest>
struct Distinct<PinList<First, Rest...>>
    : std::bool_constant<distinct_from_rest<First, Rest...> && Distinct<PinList<Rest...>>::value> {};
}  // namespace detail

// True when no physical pin appears twice in the list.
template <class List>
constexpr bool distinct_pins = detail::Distinct<List>::value;

}  // namespace vk::hal
