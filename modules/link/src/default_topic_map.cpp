#include <vk/link/topic_map.hpp>

#include <vk/msg/imu_sample.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/parameter_status.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>
#include <vk/msg/vehicle_status.hpp>
#include <vk/msg/work_item_status.hpp>

#include <array>
#include <cstddef>
#include <tuple>
#include <utility>

namespace vk::link {
namespace {

constexpr MaxRate status_rate{1.0F};

// One stream per instance of T, each at most at status_rate.
template <class T, std::size_t... I>
std::array<TopicStreamOf<T>, sizeof...(I)> make_status_streams(std::index_sequence<I...> /*unused*/)
{
    return {TopicStreamOf<T>(status_rate, I)...};
}

template <class T>
auto make_status_streams()
{
    return make_status_streams<T>(std::make_index_sequence<T::instances>{});
}

template <class A, std::size_t... I, class B, std::size_t... J>
std::array<TopicStream*, 5 + sizeof...(I) + sizeof...(J)> make_map(std::array<TopicStream*, 5> first, A& a,
                                                                   std::index_sequence<I...> /*unused*/, B& b,
                                                                   std::index_sequence<J...> /*unused*/)
{
    return {first[0], first[1], first[2], first[3], first[4], &a[I]..., &b[J]...};
}

}  // namespace

TopicMap default_topic_map()
{
    // Created on first use, after the bus storage is in place.
    static TopicStreamOf<msg::ImuSample> imu(every_sample, 0);
    static TopicStreamOf<msg::VehicleAttitude> attitude(MaxRate{50.0F}, 0);
    static TopicStreamOf<msg::VehicleAttitudeGroundtruth> truth(MaxRate{50.0F}, 0);  // SITL only
    static TopicStreamOf<msg::ParameterStatus> parameters(status_rate, 0);           // with parameter storage
    static TopicStreamOf<msg::VehicleStatus> vehicle(MaxRate{5.0F}, 0);
    static auto work_items = make_status_streams<msg::WorkItemStatus>();
    static auto links = make_status_streams<msg::LinkStatus>();
    static const auto map = make_map({&imu, &attitude, &truth, &parameters, &vehicle}, work_items, std::make_index_sequence<std::tuple_size_v<decltype(work_items)>>{}, links,
                                     std::make_index_sequence<std::tuple_size_v<decltype(links)>>{});
    return map;
}

}  // namespace vk::link
