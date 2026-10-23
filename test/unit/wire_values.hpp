// SPDX-License-Identifier: GPL-3.0-or-later
// Deterministic field values for encoding tests. test_wire_cross_language.py
// computes the same values in Python; keep the two in step.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

namespace vk::test {

// Value of element `j` of field number `k`.
template <class T>
T wire_value(std::size_t k, std::size_t j)
{
    const bool negative = (k + j) % 2 == 1;
    if constexpr (std::is_same_v<T, bool>) {
        return negative;
    } else if constexpr (std::is_floating_point_v<T>) {
        const T magnitude = static_cast<T>(k) + static_cast<T>(0.25) + static_cast<T>(j) * static_cast<T>(0.5);
        return negative ? -magnitude : magnitude;
    } else {
        std::uint64_t m = (k * 37 + j * 11 + 3) % 100;
        if constexpr (sizeof(T) == 2) {
            m = m * 300 + m;
        } else if constexpr (sizeof(T) == 4) {
            m = m * 100'000 + m;
        } else if constexpr (sizeof(T) == 8) {
            m = m * 1'000'000'000'000 + m;
        }
        auto value = static_cast<T>(m);
        if constexpr (std::is_signed_v<T>) {
            if (negative) {
                value = static_cast<T>(-value);
            }
        }
        return value;
    }
}

template <class T>
void fill_field(std::size_t k, T& field)
{
    field = wire_value<T>(k, 0);
}

template <class T, std::size_t N>
void fill_field(std::size_t k, std::array<T, N>& field)
{
    for (std::size_t j = 0; j < N; ++j) {
        field[j] = wire_value<T>(k, j);
    }
}

// Fills every field of `message` with its deterministic value.
template <class Message>
void fill(Message& message)
{
    auto fields = message.fields();
    [&]<std::size_t... K>(std::index_sequence<K...> /*unused*/) {
        (fill_field(K, std::get<K>(fields)), ...);
    }(std::make_index_sequence<std::tuple_size_v<decltype(fields)>>{});
}

}  // namespace vk::test
