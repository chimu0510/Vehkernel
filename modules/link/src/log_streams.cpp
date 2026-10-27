// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/log_streams.hpp>

#include <vk/link/log_profile_streams.hpp>
#include <vk/core/static_instance.hpp>

#include <array>

namespace vk::link {

std::span<TopicStream* const> profile_streams_for(msg::LogProfileId id, std::size_t link_instance)
{
    static std::array<core::StaticInstance<EstimatorReplayStreams>, msg::LinkStatus::instances>
        estimator_replay_instances;
    if (id == msg::LogProfileId::EstimatorReplay && link_instance < estimator_replay_instances.size()) {
        auto& inst = estimator_replay_instances.at(link_instance);
        if (!inst.has_value()) {
            inst.emplace();
        }
        return inst->streams();
    }
    return {};
}

}  // namespace vk::link
