// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/topic_map.hpp>

#include <vk/core/static_instance.hpp>

#include <vk/msg/actuator_outputs.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/allocation_status.hpp>
#include <vk/msg/body_velocity_measurement.hpp>
#include <vk/msg/body_velocity_setpoint.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/local_menu.hpp>
#include <vk/msg/parameter_status.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>
#include <vk/msg/vehicle_local_odometry.hpp>
#include <vk/msg/vehicle_local_odometry_groundtruth.hpp>
#include <vk/msg/vehicle_status.hpp>
#include <vk/msg/wheel_encoders.hpp>
#include <vk/msg/work_item_status.hpp>

#include <array>
#include <cstddef>
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

// One Link's streams: each stream keeps its own subscriptions and rates, so
// every Link needs its own (scale_to_baud changes a stream's rate).
struct DefaultStreams {
    explicit DefaultStreams(float imu_rate_hz) : imu(every_sample, 0, imu_rate_hz) {}

    TopicStreamOf<msg::ImuSample> imu;
    TopicStreamOf<msg::VehicleAttitude> attitude{MaxRate{50.0F}, 0};
    TopicStreamOf<msg::VehicleAttitudeGroundtruth> truth{MaxRate{50.0F}, 0};  // SITL only
    TopicStreamOf<msg::VehicleLocalOdometryGroundtruth> odometry_truth{MaxRate{50.0F}, 0};  // SITL only
    TopicStreamOf<msg::ParameterStatus> parameters{status_rate, 0};           // with parameter storage
    TopicStreamOf<msg::VehicleStatus> vehicle{MaxRate{5.0F}, 0};
    TopicStreamOf<msg::WheelEncoders> wheels{MaxRate{50.0F}, 0};
    TopicStreamOf<msg::ActuatorOutputs> outputs{MaxRate{50.0F}, 0};
    TopicStreamOf<msg::AllocationStatus> allocation{MaxRate{50.0F}, 0};  // with the control chain
    TopicStreamOf<msg::BodyVelocityMeasurement> measurement{MaxRate{50.0F}, 0};
    TopicStreamOf<msg::VehicleLocalOdometry> odometry{MaxRate{50.0F}, 0};
    TopicStreamOf<msg::ActuatorSetpoint> setpoint{Inbound{.setpoint = true}, 0};
    TopicStreamOf<msg::BodyVelocitySetpoint> body_setpoint{Inbound{.setpoint = true}, 0};
    TopicStreamOf<msg::LocalMenu> menu{MaxRate{5.0F}, 0};  // with the local menu
    std::array<TopicStreamOf<msg::WorkItemStatus>, msg::WorkItemStatus::instances> work_items =
        make_status_streams<msg::WorkItemStatus>();
    std::array<TopicStreamOf<msg::LinkStatus>, msg::LinkStatus::instances> links = make_status_streams<msg::LinkStatus>();
    std::array<TopicStream*, 14> first{&imu,         &attitude, &truth,       &odometry_truth, &parameters,
                                       &vehicle,     &wheels,   &outputs,     &allocation,     &measurement,
                                       &odometry,    &setpoint, &body_setpoint, &menu};
    std::array<TopicStream*, 14 + msg::WorkItemStatus::instances + msg::LinkStatus::instances> map =
        make_map(first, std::make_index_sequence<14>{}, work_items,
                 std::make_index_sequence<msg::WorkItemStatus::instances>{}, links,
                 std::make_index_sequence<msg::LinkStatus::instances>{});
};

}  // namespace

TopicMap default_topic_map(float imu_rate_hz, std::size_t link)
{
    // Created on first use, after the bus storage is in place.
    static std::array<core::StaticInstance<DefaultStreams>, msg::LinkStatus::instances> streams;
    auto& instance = streams.at(link);
    if (!instance.has_value()) {
        instance.emplace(imu_rate_hz);
    }
    return instance->map;
}

}  // namespace vk::link
