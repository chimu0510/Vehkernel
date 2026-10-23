// SPDX-License-Identifier: GPL-3.0-or-later
// The MCU side of a Link session (ADR-0002): the handshake, the peer timeout
// and time synchronization replies. Independent of the transport and of
// FreeRTOS, so it runs on the development host in tests.
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
// Parameters (ADR-0012) are read and changed through the parameter runtime:
// param_get and param_set are answered at once with param_value or
// param_reject, and param_list_request starts a stream of param_value frames
// that send_pending() paces after the topic_info frames.
//
// Mode, arming and emergency stop requests (ADR-0005) go to the vehicle
// control service, with this Link as their source, and are answered at once:
// mode_request with mode_reply or mode_reject, arming_request with
// arming_reply, which carries the arming checks skipped, or arming_reject,
// and estop_request with estop_reply. bringup_run is refused at once with
// bringup_reject, or accepted silently: its bringup_result follows when the
// case ends. Without the service they go unanswered.
#pragma once

#include <vk/core/parameter_storage.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/link/frame.hpp>
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

// Where a session sends the frames it writes.
class FrameSink {
public:
    FrameSink() = default;
    virtual ~FrameSink() = default;
    FrameSink(const FrameSink&) = delete;
    FrameSink& operator=(const FrameSink&) = delete;
    FrameSink(FrameSink&&) = delete;
    FrameSink& operator=(FrameSink&&) = delete;

    // Sends one frame, as encoded for the wire; false if it was dropped.
    virtual bool send(std::span<const std::uint8_t> wire) = 0;
};

// The parameters a session offers; without a runtime it offers none.
struct ParameterAccess {
    core::Parameters* parameters = nullptr;
    std::uint64_t set_hash = 0;                         // of the metadata the parameters come from
    const core::ParameterStorage* storage = nullptr;    // for the flags of param_value, if parameters are stored
};

// The vehicle control a session offers; without it, mode, arming and
// emergency stop requests go unanswered.
struct ControlAccess {
    core::VehicleControl* control = nullptr;
    core::ControlSource source = 0;                     // this Link, as the source of its requests
    void (*set_sim_actuator_power)(bool) = nullptr;     // SITL only: sim_actuator_power frames
    void (*set_sim_user_key)(bool) = nullptr;           // SITL only: sim_user_key frames
};

class Session {
public:
    static constexpr std::uint64_t default_peer_timeout_us = 3'000'000;

    explicit Session(std::span<const TopicDescription> topics = schema_topics,
                     std::uint64_t peer_timeout_us = default_peer_timeout_us, ParameterAccess parameters = {},
                     ControlAccess control = {})
        : topics_(topics), peer_timeout_us_(peer_timeout_us), parameters_(parameters), control_(control)
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

    // Sends the result of a bring-up case this session started.
    bool send_bringup_result(const msg::BringupStatus& status, FrameSink& sink);

    // Whether topic_info frames of the handshake are still to be sent.
    [[nodiscard]] bool describing() const { return info_next_ < topics_.size(); }

    // Whether a parameter list is still being sent.
    [[nodiscard]] bool listing() const { return list_next_ < parameter_count(); }

    [[nodiscard]] std::uint32_t frames_sent() const { return frames_sent_; }

private:
    void on_hello(BodyReader body, FrameSink& sink);
    void on_time_sync(BodyReader body, std::uint64_t now_us, FrameSink& sink);
    void on_param_list(BodyReader body);
    bool send_topic_info(const TopicDescription& topic, FrameSink& sink);
    void on_param_get(BodyReader body, FrameSink& sink);
    void on_param_set(BodyReader body, FrameSink& sink);
    bool send_param_value(std::size_t index, FrameSink& sink);
    void send_param_reject(core::ParamResult reason, std::string_view name, FrameSink& sink);
    void on_mode_request(BodyReader body, FrameSink& sink);
    void on_arming_request(BodyReader body, FrameSink& sink);
    void on_estop_request(BodyReader body, FrameSink& sink);
    void on_bringup_run(BodyReader body, FrameSink& sink);
    void on_reboot_request(BodyReader body, FrameSink& sink);
    void on_sim_actuator_power(BodyReader body);
    void on_sim_user_key(BodyReader body);
    [[nodiscard]] std::size_t parameter_count() const
    {
        return parameters_.parameters == nullptr ? 0 : parameters_.parameters->count();
    }
    bool send(FrameType type, std::span<const std::uint8_t> body, FrameSink& sink);

    std::span<const TopicDescription> topics_;
    std::uint64_t peer_timeout_us_;
    ParameterAccess parameters_;
    ControlAccess control_;
    // Next topic to describe with topic_info; none at or past the topic count.
    std::size_t info_next_ = std::numeric_limits<std::size_t>::max();
    // Next parameter of the list being sent; none at or past the count.
    std::size_t list_next_ = std::numeric_limits<std::size_t>::max();
    FrameWriter writer_;
    std::uint8_t version_ = 0;
    bool handshake_done_ = false;
    std::uint64_t last_heard_us_ = 0;
    std::uint32_t frames_sent_ = 0;
};

}  // namespace vk::link
