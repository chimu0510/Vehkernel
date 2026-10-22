#include <vk/link/topic_map.hpp>

#include <vk/msg/actuator_outputs.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/local_menu.hpp>
#include <vk/msg/parameter_status.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>
#include <vk/msg/vehicle_status.hpp>
#include <vk/msg/wheel_encoders.hpp>
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

template <std::size_t N, std::size_t... F, class A, std::size_t... I, class B, std::size_t... J>
std::array<TopicStream*, N + sizeof...(I) + sizeof...(J)> make_map(const std::array<TopicStream*, N>& first,
                                                                   std::index_sequence<F...> /*unused*/, A& a,
                                                                   std::index_sequence<I...> /*unused*/, B& b,
                                                                   std::index_sequence<J...> /*unused*/)
{
    return {first[F]..., &a[I]..., &b[J]...};
}

}  // namespace

TopicMap default_topic_map(float imu_rate_hz)
{
    // Created on first use, after the bus storage is in place.
    static TopicStreamOf<msg::ImuSample> imu(every_sample, 0, imu_rate_hz);
    static TopicStreamOf<msg::VehicleAttitude> attitude(MaxRate{50.0F}, 0);
    static TopicStreamOf<msg::VehicleAttitudeGroundtruth> truth(MaxRate{50.0F}, 0);  // SITL only
    static TopicStreamOf<msg::ParameterStatus> parameters(status_rate, 0);           // with parameter storage
    static TopicStreamOf<msg::VehicleStatus> vehicle(MaxRate{5.0F}, 0);
    static TopicStreamOf<msg::WheelEncoders> wheels(MaxRate{50.0F}, 0);
    static TopicStreamOf<msg::ActuatorOutputs> outputs(MaxRate{50.0F}, 0);
    static TopicStreamOf<msg::ActuatorSetpoint> setpoint(Inbound{.setpoint = true}, 0);
    static TopicStreamOf<msg::LocalMenu> menu(MaxRate{5.0F}, 0);                       // with the local menu
    static auto work_items = make_status_streams<msg::WorkItemStatus>();
    static auto links = make_status_streams<msg::LinkStatus>();
    static const std::array<TopicStream*, 9> first{&imu,    &attitude, &truth,    &parameters, &vehicle,
                                                   &wheels, &outputs,  &setpoint, &menu};
    static const auto map =
        make_map(first, std::make_index_sequence<std::tuple_size_v<decltype(first)>>{}, work_items,
                 std::make_index_sequence<std::tuple_size_v<decltype(work_items)>>{}, links,
                 std::make_index_sequence<std::tuple_size_v<decltype(links)>>{});
    return map;
}

}  // namespace vk::link
