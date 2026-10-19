// The MCU side of a Link session (ADR-0002): the handshake, the peer timeout
// and time synchronization replies. Independent of the transport and of
// FreeRTOS, so it runs on the development host in tests.
//
// The development host starts with hello. The MCU accepts it when it offers
// the MCU's protocol version and answers with hello_reply, carrying the
// parameter set hash and count, then one topic_info frame per schema topic
// with its ID, name and schema hash; the host compares the hashes with its
// own schema. Topic data may flow only while the session is connected: after
// a successful handshake and as long as the peer was heard within the
// timeout. The host keeps the session alive with time sync requests.
//
// Parameters (ADR-0012) are read and changed through the parameter runtime:
// param_get and param_set are answered at once with param_value or
// param_reject, and param_list_request starts a stream of param_value frames
// that send_pending() paces.
#pragma once

#include <vk/core/parameters.hpp>
#include <vk/link/frame.hpp>
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
    std::uint64_t set_hash = 0;  // of the metadata the parameters come from
};

class Session {
public:
    static constexpr std::uint64_t default_peer_timeout_us = 3'000'000;

    explicit Session(std::span<const TopicDescription> topics = schema_topics,
                     std::uint64_t peer_timeout_us = default_peer_timeout_us, ParameterAccess parameters = {})
        : topics_(topics), peer_timeout_us_(peer_timeout_us), parameters_(parameters)
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

    // Sends one topic data frame; false if it did not fit, was dropped, or the session is not connected.
    bool send_topic(std::uint16_t topic_id, std::uint8_t instance, std::span<const std::uint8_t> message,
                    std::uint64_t now_us, FrameSink& sink);

    // Sends up to `max_frames` frames of a parameter list in progress. A frame
    // the sink drops is sent again on the next call.
    void send_pending(std::uint64_t now_us, FrameSink& sink, std::size_t max_frames);

    // Whether a parameter list is still being sent.
    [[nodiscard]] bool listing() const { return list_next_ < parameter_count(); }

    [[nodiscard]] std::uint32_t frames_sent() const { return frames_sent_; }

private:
    void on_hello(BodyReader body, FrameSink& sink);
    void on_time_sync(BodyReader body, std::uint64_t now_us, FrameSink& sink);
    void on_param_list(BodyReader body);
    void on_param_get(BodyReader body, FrameSink& sink);
    void on_param_set(BodyReader body, FrameSink& sink);
    bool send_param_value(std::size_t index, FrameSink& sink);
    void send_param_reject(core::ParamResult reason, std::string_view name, FrameSink& sink);
    [[nodiscard]] std::size_t parameter_count() const
    {
        return parameters_.parameters == nullptr ? 0 : parameters_.parameters->count();
    }
    bool send(FrameType type, std::span<const std::uint8_t> body, FrameSink& sink);

    std::span<const TopicDescription> topics_;
    std::uint64_t peer_timeout_us_;
    ParameterAccess parameters_;
    // Next parameter of the list being sent; none at or past the count.
    std::size_t list_next_ = std::numeric_limits<std::size_t>::max();
    FrameWriter writer_;
    std::uint8_t version_ = 0;
    bool handshake_done_ = false;
    std::uint64_t last_heard_us_ = 0;
    std::uint32_t frames_sent_ = 0;
};

}  // namespace vk::link
