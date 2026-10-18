#include <vk/link/topic_map.hpp>

#include <vk/core/work_queue.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/work_item_status.hpp>

#include <array>
#include <cstddef>
#include <utility>

namespace vk::link {
namespace {

constexpr MaxRate status_rate{1.0F};

template <std::size_t... I>
std::array<TopicStreamOf<msg::WorkItemStatus>, sizeof...(I)> make_status_streams(
    std::index_sequence<I...> /*unused*/)
{
    return {TopicStreamOf<msg::WorkItemStatus>(status_rate, I)...};
}

template <class Streams, std::size_t... I>
std::array<TopicStream*, sizeof...(I) + 1> make_map(TopicStream& first, Streams& rest,
                                                    std::index_sequence<I...> /*unused*/)
{
    return {&first, &rest[I]...};
}

}  // namespace

TopicMap default_topic_map()
{
    // Created on first use, after the bus storage is in place.
    static TopicStreamOf<msg::ImuSample> imu(every_sample, 0);
    static auto statuses = make_status_streams(std::make_index_sequence<core::max_work_items>{});
    static const auto map = make_map(imu, statuses, std::make_index_sequence<core::max_work_items>{});
    return map;
}

}  // namespace vk::link
