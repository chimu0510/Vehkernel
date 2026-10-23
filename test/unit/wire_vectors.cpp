// SPDX-License-Identifier: GPL-3.0-or-later
// Prints every schema message, filled with deterministic values and encoded,
// as "<topic name> <hex bytes>" lines for test_wire_cross_language.py.
#include "wire_values.hpp"

#include <vk/msg/topics.hpp>

#include <array>
#include <cstdint>
#include <cstdio>

namespace {

template <class Message>
void print_vector()
{
    Message message{};
    vk::test::fill(message);
    std::array<std::uint8_t, Message::encoded_size> bytes{};
    const std::size_t size = message.encode(bytes);
    std::printf("%.*s ", static_cast<int>(Message::topic_name.size()), Message::topic_name.data());
    for (std::size_t i = 0; i < size; ++i) {
        std::printf("%02x", bytes.at(i));
    }
    std::printf("\n");
}

template <class... Messages>
void print_all(vk::msg::TopicList<Messages...> /*unused*/)
{
    (print_vector<Messages>(), ...);
}

}  // namespace

int main()
{
    print_all(vk::msg::AllTopics{});
    return 0;
}
