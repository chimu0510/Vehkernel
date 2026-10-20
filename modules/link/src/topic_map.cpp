#include <vk/link/frame.hpp>
#include <vk/link/topic_map.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace vk::link {
namespace {

// The most streams a map scales; the default map has far fewer.
constexpr std::size_t max_streams = 64;

// Bytes on the wire for one topic data frame of a stream's messages.
float wire_bytes(const TopicStream& stream)
{
    const std::size_t frame = header_size + 3 + stream.message_size() + crc_size;
    return static_cast<float>(cobs_max_encoded_size(frame) + 1);
}

}  // namespace

float topic_map_load(TopicMap map)
{
    float load = 0.0F;
    for (const TopicStream* stream : map) {
        load += stream->rate_hz() * wire_bytes(*stream);
    }
    return load;
}

std::uint8_t scale_to_baud(TopicMap map, std::uint32_t baud)
{
    if (baud == 0 || map.size() > max_streams) {
        return 100;
    }
    const float capacity = static_cast<float>(baud) / 10.0F * max_port_load;
    float fixed = 0.0F;     // streams that keep their rate or sit at the 1 Hz floor
    float scalable = 0.0F;  // streams the common factor applies to
    std::array<bool, max_streams> floored{};
    for (std::size_t i = 0; i < map.size(); ++i) {
        const float bytes = map[i]->nominal_hz() * wire_bytes(*map[i]);
        floored[i] = map[i]->nominal_hz() <= 1.0F;
        (floored[i] ? fixed : scalable) += bytes;
    }
    if (fixed + scalable <= capacity) {
        return 100;
    }
    // A stream the factor would take below 1 Hz stays at 1 Hz, which leaves
    // less for the others; repeat until no other stream drops below.
    float factor = 0.0F;
    for (bool changed = true; changed;) {
        changed = false;
        factor = scalable > 0.0F ? std::max(0.0F, (capacity - fixed) / scalable) : 0.0F;
        for (std::size_t i = 0; i < map.size(); ++i) {
            if (!floored[i] && map[i]->nominal_hz() * factor < 1.0F) {
                floored[i] = true;
                scalable -= map[i]->nominal_hz() * wire_bytes(*map[i]);
                fixed += wire_bytes(*map[i]);
                changed = true;
            }
        }
    }
    for (std::size_t i = 0; i < map.size(); ++i) {
        if (map[i]->nominal_hz() > 1.0F) {
            map[i]->limit_rate(floored[i] ? 1.0F : map[i]->nominal_hz() * factor);
        }
    }
    return static_cast<std::uint8_t>(std::clamp(std::floor(factor * 100.0F), 0.0F, 100.0F));
}

}  // namespace vk::link
