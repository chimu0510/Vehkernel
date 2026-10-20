// A Link (ADR-0002): a topic map, the custom protocol and a transport, run as
// one work item on wq:io (ADR-0008).
//
// Each run drains the bytes the transport has received through the protocol,
// which answers the handshake, time sync, parameter, mode, arming and
// emergency stop requests, then sends a few frames of the handshake's
// topic_info frames or of a parameter list in progress, and every topic map
// entry that is due once the session is connected and every topic has been
// described. A frame the transport cannot
// take is dropped and counted; the Link never blocks its work queue. Once a
// second it publishes its statistics on link_status. On a transport with a
// baud rate, start() scales the topic map down to fit it (ADR-0002).
#pragma once

#include <vk/core/work_queue.hpp>
#include <vk/hal/transport.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/session.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/msg/link_status.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace vk::link {

enum class HandshakeState : std::uint8_t {
    Waiting = 0,     // no handshake yet
    Connected = 1,   // handshake done, peer heard within the timeout
    PeerSilent = 2,  // handshake done, peer not heard within the timeout
};

// Statistics of one Link since boot.
struct LinkStats {
    std::uint32_t frames_sent;
    std::uint32_t bytes_sent;
    std::uint32_t frames_received;  // with a valid CRC
    std::uint32_t bytes_received;
    std::uint32_t drops;            // frames the transport could not take
    std::uint32_t crc_errors;
    std::uint32_t malformed;        // invalid COBS, too short or too long
    std::uint32_t lost;             // frames missing from the peer's sequence
    HandshakeState handshake_state;
    std::uint8_t rate_scale;        // percent of the topic map's rates, scaled to fit the baud rate
};

class Link final : public core::WorkItem {
public:
    static constexpr std::uint32_t default_interval_us = 5'000;
    static constexpr std::uint32_t status_interval_us = 1'000'000;

    struct Config {
        std::uint8_t instance = 0;                        // link_status instance
        std::uint32_t interval_us = default_interval_us;  // how often the Link runs
        std::uint64_t peer_timeout_us = Session::default_peer_timeout_us;
        ParameterAccess parameters{};                     // what the peer may read and change
        ControlAccess control{};                          // modes, arming and the emergency stop; the source is the instance
    };

    // topic_info and parameter list frames sent per run: at the default
    // interval, 800 per second. With hello_reply, a handshake's run writes at
    // most five frames, well within a 1 KiB transmit buffer.
    static constexpr std::size_t paced_frames_per_run = 4;

    Link(std::string_view name, hal::Transport& transport, TopicMap map, const Config& config,
         core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

    // From the Link's own work queue, or once the scheduler has stopped.
    [[nodiscard]] LinkStats link_stats() const;

protected:
    void run() override;

private:
    // Writes whole frames to the transport, or drops them.
    class TransportSink final : public FrameSink {
    public:
        explicit TransportSink(hal::Transport& transport) : transport_(&transport) {}

        bool send(std::span<const std::uint8_t> wire) override;

        std::uint32_t frames = 0;
        std::uint32_t bytes = 0;
        std::uint32_t drops = 0;

    private:
        hal::Transport* transport_;
    };

    void receive(std::uint64_t now_us);
    void send_topics(std::uint64_t now_us);
    void publish_status(std::uint64_t now_us);

    hal::Transport* transport_;
    TopicMap map_;
    Config config_;
    Session session_;
    FrameReader reader_;
    TransportSink sink_;
    core::Publication<msg::LinkStatus> status_publication_;
    std::uint32_t bytes_received_ = 0;
    HandshakeState handshake_state_ = HandshakeState::Waiting;
    std::uint64_t next_status_us_ = 0;
    std::uint8_t rate_scale_ = 100;
    std::array<std::uint8_t, 2 * max_wire_size> receive_buffer_{};
};

}  // namespace vk::link
