// SPDX-License-Identifier: GPL-3.0-or-later
// Generated encoders and decoders (ADR-0006): packed little-endian, fields in
// schema order, and every schema message round-trips.
#include "wire_values.hpp"

#include <vk/msg/imu_sample.hpp>
#include <vk/msg/topics.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

namespace {

using vk::msg::ImuSample;

TEST(Wire, EncodesPackedLittleEndianInSchemaOrder)
{
    ImuSample sample{};
    sample.timestamp = 0x0102030405060708ULL;
    sample.device_instance = 0xAB;
    sample.accel = {1.0F, 0.0F, 0.0F};
    std::array<std::uint8_t, ImuSample::encoded_size> bytes{};
    ASSERT_EQ(sample.encode(bytes), ImuSample::encoded_size);
    const std::vector<std::uint8_t> head(bytes.begin(), bytes.begin() + 13);
    // timestamp (8, little-endian), device_instance (1, no padding), accel[0] = 1.0f.
    EXPECT_EQ(head, (std::vector<std::uint8_t>{0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0xAB, 0x00, 0x00,
                                               0x80, 0x3F}));
}

TEST(Wire, EncodedSizeHasNoPadding)
{
    // uint64 + uint8 + 7 float32 = 37 bytes, though the struct is padded.
    EXPECT_EQ(ImuSample::encoded_size, 37U);
    EXPECT_GT(sizeof(ImuSample), ImuSample::encoded_size);
}

TEST(Wire, RefusesBuffersOfTheWrongSize)
{
    ImuSample sample{};
    std::array<std::uint8_t, ImuSample::encoded_size - 1> small{};
    EXPECT_EQ(sample.encode(small), 0U);
    std::array<std::uint8_t, ImuSample::encoded_size + 1> large{};
    EXPECT_FALSE(ImuSample::decode(large, sample));
}

template <class Message>
void expect_round_trip()
{
    Message original{};
    vk::test::fill(original);
    std::array<std::uint8_t, Message::encoded_size> bytes{};
    ASSERT_EQ(original.encode(bytes), Message::encoded_size);
    Message decoded{};
    ASSERT_TRUE(Message::decode(bytes, decoded));
    EXPECT_EQ(decoded.fields(), original.fields()) << Message::topic_name;
}

template <class... Messages>
void expect_all_round_trip(vk::msg::TopicList<Messages...> /*unused*/)
{
    (expect_round_trip<Messages>(), ...);
}

TEST(Wire, EverySchemaMessageRoundTrips)
{
    expect_all_round_trip(vk::msg::AllTopics{});
}

}  // namespace
