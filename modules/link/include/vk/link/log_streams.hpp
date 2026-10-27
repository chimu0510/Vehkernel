// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <vk/link/topic_map.hpp>
#include <vk/msg/log_profiles.hpp>

#include <span>

namespace vk::link {

// Returns the streams for a profile, or empty span if unknown.
std::span<TopicStream* const> profile_streams_for(msg::LogProfileId id, std::size_t link_instance = 0);

}  // namespace vk::link
