// SPDX-License-Identifier: GPL-3.0-or-later
// The MCU side of a Link session (ADR-0002): the handshake, the peer timeout,
// time synchronization replies and topic data. Independent of the transport
// and of FreeRTOS, so it runs on the development host in tests.
//
// The development host starts with hello. The MCU accepts it when it offers
// the MCU's protocol version and answers with hello_reply, carrying the
// parameter set hash and count, then one topic_info frame per schema topic
// with its ID, name and schema hash; the host compares the hashes with its
// own schema. The topic_info frames are paced by send_pending(), so a
// handshake never asks the transport for more than a few frames at once.
// Topic data may flow only while the session is connected, after a
// successful handshake and as long as the peer was heard within the
// timeout, and once every topic_info has gone out, so the host never sees
// data for a topic it has not been told about. The host keeps the session
// alive with time sync requests.
//
// Once the handshake has succeeded, the session hands the other frames it
// receives to a handler by frame type: parameter frames to its
// ParameterHandler (ADR-0012), whose param_value list send_pending() paces
// after the topic_info frames, vehicle control frames to its ControlHandler
// (ADR-0005), and the SITL-only frames to the FrameHandler it is given, if
// any (SimHandler, ADR-0018). Without one, a session ignores those frames.
#pragma once

#include <vk/link/control_handler.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/frame_handler.hpp>
#include <vk/link/parameter_handler.hpp>
#include <vk/msg/bringup_status.hpp>
#include <vk/msg/topics.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>

namespace vk::link {

struct TopicDescription {
    std::uint16_t id;
    std::uint8_t instances;
    std::uint64_t schema_hash;
    std::string_view name;
};

namespace detail {
template <class... Messages>
constexpr std::array<TopicDescription, sizeof...(Messages)> describe(msg::TopicList<Messages...> /*unused*/)
{
    return {TopicDescription{Messages::topic_id, static_cast<std::uint8_t>(Messages::instances),
                             Messages::schema_hash, Messages::topic_name}...};
}
}  // namespace detail

// Every topic of the schema, in topic ID order.
inline constexpr auto schema_topics = detail::describe(msg::AllTopics{});

class Session {
public:
    static constexpr std::uint64_t default_peer_timeout_us = 3'000'000;

    // `sim_frames` handles the SITL-only frames; only SITL gives one.
    explicit Session(std::span<const TopicDescription> topics = schema_topics,
                     std::uint64_t peer_timeout_us = default_peer_timeout_us, ParameterAccess parameters = {},
                     ControlAccess control = {}, FrameHandler* sim_frames = nullptr)
        : topics_(topics), peer_timeout_us_(peer_timeout_us), parameters_(parameters), control_(control),
          sim_frames_(sim_frames)
    {
    }

    // Handles one frame received from the peer at `now_us`.
    void handle(const Frame& frame, std::uint64_t now_us, FrameSink& sink);

    // Whether topic data may be sent at `now_us`.
    [[nodiscard]] bool connected(std::uint64_t now_us) const;

    // Whether a handshake has succeeded, even if the peer fell silent since.
    [[nodiscard]] bool handshake_done() const { return handshake_done_; }

    // The negotiated protocol version, 0 before a successful handshake.
    [[nodiscard]] std::uint8_t version() const { return version_; }

    // Sends one topic data frame; false if it did not fit, was dropped, the
    // session is not connected or topic_info frames are still to go out.
    bool send_topic(std::uint16_t topic_id, std::uint8_t instance, std::span<const std::uint8_t> message,
                    std::uint64_t now_us, FrameSink& sink);

    // Sends up to `max_frames` frames of the handshake's topic_info frames,
    // then of a parameter list in progress. A frame the sink drops is sent
    // again on the next call.
    void send_pending(std::uint64_t now_us, FrameSink& sink, std::size_t max_frames);

    // Sends the replies to the vehicle control requests the commander has
    // answered since the last call (ControlHandler).
    void send_answers(std::uint64_t now_us, FrameSink& sink);

    // Tells the commander the peer fell silent.
    void source_lost(std::uint64_t now_us);

    // Sends the result of a bring-up case this session started.
    bool send_bringup_result(const msg::BringupStatus& status, FrameSink& sink);

    // Whether topic_info frames of the handshake are still to be sent.
    [[nodiscard]] bool describing() const { return info_next_ < topics_.size(); }

    // Whether a parameter list is still being sent.
    [[nodiscard]] bool listing() const { return parameters_.listing(); }

    [[nodiscard]] std::uint32_t frames_sent() const { return frames_sent_; }

private:
    void on_hello(BodyReader body, FrameSender& out);
    void on_time_sync(BodyReader body, std::uint64_t now_us, FrameSender& out);
    bool send_topic_info(const TopicDescription& topic, FrameSender& out);
    FrameSender sender(FrameSink& sink) { return {writer_, sink, frames_sent_}; }

    std::span<const TopicDescription> topics_;
    std::uint64_t peer_timeout_us_;
    ParameterHandler parameters_;
    ControlHandler control_;
    FrameHandler* sim_frames_;  // SITL only
    // Next topic to describe with topic_info; none at or past the topic count.
    std::size_t info_next_ = std::numeric_limits<std::size_t>::max();
    FrameWriter writer_;
    std::uint8_t version_ = 0;
    bool handshake_done_ = false;
    std::uint64_t last_heard_us_ = 0;
    std::uint32_t frames_sent_ = 0;
};

}  // namespace vk::link
