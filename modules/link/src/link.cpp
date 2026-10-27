// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/link.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>

namespace vk::link {
namespace {

// Reads per run; bounds the run time when bytes keep arriving.
constexpr int max_reads_per_run = 8;

// A Link's requests come from the Link itself.
ControlAccess with_source(ControlAccess control, std::uint8_t instance)
{
    control.source = instance;
    return control;
}

std::atomic<const Link*> active_logger{nullptr};

}  // namespace

bool Link::is_profile_stream(std::uint16_t topic_id, std::uint8_t instance, void* context)
{
    const auto* link = static_cast<const Link*>(context);
    return link != nullptr && link->is_profile_topic(topic_id, instance);
}

void Link::LogHandler::handle(FrameType type, BodyReader body, FrameSender& out)
{
    if (type == FrameType::LogStart) {
        link_->on_log_start(body, link_->current_run_now_us_, out);
    } else if (type == FrameType::LogStop) {
        link_->on_log_stop();
    }
}

Link::Link(std::string_view name, hal::Transport& transport, TopicMap map, const Config& config,
           core::WorkQueue& queue)
    : WorkItem(name, queue), transport_(&transport), map_(map), config_(config),
      session_(schema_topics, config.peer_timeout_us, config.parameters, with_source(config.control, config.instance),
               config.sim_frames, &log_handler_),
      sink_(transport), status_publication_(config.instance)
{
}

Link::~Link()
{
    on_log_stop();
}

void Link::start()
{
    rate_scale_ = scale_to_baud(map_, transport_->baud_rate());
    schedule_periodic(config_.interval_us);
}

LinkStats Link::link_stats() const
{
    const ReaderStats& received = reader_.stats();
    return {.frames_sent = sink_.frames,
            .bytes_sent = sink_.bytes,
            .frames_received = received.frames,
            .bytes_received = bytes_received_,
            .drops = sink_.drops,
            .crc_errors = received.crc_errors,
            .malformed = received.malformed,
            .lost = received.lost,
            .handshake_state = handshake_state_,
            .rate_scale = rate_scale_,
            .inbound_undeclared = inbound_undeclared_};
}

void Link::run()
{
    const std::uint64_t now = queue().now_us();
    current_run_now_us_ = now;
    receive(now);
    const HandshakeState previous = handshake_state_;
    if (session_.connected(now)) {
        handshake_state_ = HandshakeState::Connected;
    } else {
        handshake_state_ = session_.handshake_done() ? HandshakeState::PeerSilent : HandshakeState::Waiting;
        stop_logging();
    }
    follow_control(previous, now);
    session_.send_answers(now, sink_);
    publish_status(now);
    session_.send_pending(now, sink_, paced_frames_per_run);
    send_topics(now);
}

void Link::publish_status(std::uint64_t now_us)
{
    if (now_us < next_status_us_) {
        return;
    }
    next_status_us_ = now_us + status_interval_us;
    const LinkStats current = link_stats();
    msg::LinkStatus status{};
    status.timestamp = now_us;
    status.frames_sent = current.frames_sent;
    status.bytes_sent = current.bytes_sent;
    status.frames_received = current.frames_received;
    status.bytes_received = current.bytes_received;
    status.drops = current.drops;
    status.crc_errors = current.crc_errors;
    status.malformed = current.malformed;
    status.lost = current.lost;
    status.handshake_state = static_cast<std::uint8_t>(current.handshake_state);
    status.rate_scale = current.rate_scale;
    status.inbound_undeclared = current.inbound_undeclared;
    status_publication_.publish(status);
}

void Link::receive(std::uint64_t now_us)
{
    for (int i = 0; i < max_reads_per_run; ++i) {
        const std::size_t size = transport_->read(receive_buffer_);
        if (size == 0) {
            return;
        }
        bytes_received_ += size;
        // Frames are handled read by read, so a hello selects the peer it came from.
        reader_.feed(std::span(receive_buffer_).first(size), [&](const Frame& frame) {
            if (frame.type == FrameType::Hello) {
                transport_->select_peer();
            }
            session_.handle(frame, now_us, sink_);
            if (frame.type == FrameType::TopicData && session_.handshake_done()) {
                on_topic_data(BodyReader(frame.body), now_us);
            }
        });
    }
}

void Link::follow_control(HandshakeState previous, std::uint64_t now_us)
{
    if (!config_.control.enabled) {
        return;
    }
    if (previous == HandshakeState::Connected && handshake_state_ == HandshakeState::PeerSilent) {
        session_.source_lost(now_us);
    }
    msg::BringupStatus status{};
    while (bringup_status_.copy(status)) {
        if (status.source != config_.instance) {
            continue;
        }
        if (status.state == 1) {  // running: this Link's case; its result follows
            bringup_started_ = true;
        } else if (status.state >= 2 && bringup_started_) {  // done or aborted
            bringup_started_ = false;
            bringup_result_ = status;
            bringup_result_due_ = true;
        }
    }
    if (bringup_result_due_) {
        // A result the transport dropped goes again next run.
        bringup_result_due_ = !session_.send_bringup_result(bringup_result_, sink_);
    }
}

void Link::on_topic_data(BodyReader body, std::uint64_t now_us)
{
    const std::uint16_t topic_id = body.u16();
    const std::uint8_t instance = body.u8();
    const auto message = body.rest();
    if (!body.ok()) {
        return;
    }
    const auto stream = std::ranges::find_if(map_, [&](const TopicStream* s) {
        return s->inbound() && s->topic_id() == topic_id && s->instance() == instance;
    });
    if (stream == map_.end()) {
        ++inbound_undeclared_;
        return;
    }
    // A setpoint is tagged with this Link as its source; its consumers judge
    // whether it counts (setpoint arbitration, ADR-0005).
    (*stream)->receive(message, now_us, config_.instance);
}

void Link::on_log_start(BodyReader body, std::uint64_t now_us, FrameSender& /*out*/)
{
    const std::uint32_t profile_mask = body.u32();

    // 1. Refused when the vehicle is armed
    msg::ArmingState arming{};
    while (arming_.copy(arming)) {
        armed_ = arming.armed;
    }
    if (armed_) {
        session_.send_log_info(LogStartResult::Armed, config_.board_info, 0.0F, sink_);
        return;
    }

    // 2. Refused when the profile is unknown
    const auto* profile = msg::find_log_profile(static_cast<msg::LogProfileId>(profile_mask));
    if (profile == nullptr || profile_mask == 0) {
        session_.send_log_info(LogStartResult::UnknownProfile, config_.board_info, 0.0F, sink_);
        return;
    }

    // 3. Refused when another Link is already logging
    const Link* current_logger = active_logger.load(std::memory_order_acquire);
    if (current_logger != nullptr && current_logger != this) {
        session_.send_log_info(LogStartResult::AlreadyLogging, config_.board_info, profile->bandwidth, sink_);
        return;
    }

    // 4. Refused when the profile is over 80 % of the port together with the topic map
    const std::uint32_t baud = transport_->baud_rate();
    if (baud != 0) {
        const float capacity = static_cast<float>(baud) / 10.0F * max_port_load;
        float fixed = 0.0F;
        for (const TopicStream* stream : map_) {
            if (is_profile_topic(stream->topic_id(), stream->instance())) {
                continue;  // Suppressed while logging: left out of the port's load
            }
            if (stream->nominal_hz() <= 1.0F) {
                const std::size_t frame_sz = header_size + 3 + stream->message_size() + crc_size;
                fixed += stream->nominal_hz() * static_cast<float>(cobs_max_encoded_size(frame_sz) + 1);
            }
        }
        if (profile->bandwidth + fixed > capacity) {
            session_.send_log_info(LogStartResult::OverBudget, config_.board_info, profile->bandwidth, sink_);
            return;
        }
    }

    // Claim the active logger
    const Link* expected = nullptr;
    if (!active_logger.compare_exchange_strong(expected, this, std::memory_order_acq_rel)) {
        if (expected != this) {
            session_.send_log_info(LogStartResult::AlreadyLogging, config_.board_info, profile->bandwidth, sink_);
            return;
        }
    }

    // Accepted
    logging_ = true;
    active_profile_ = profile;
    profile_streams_ = profile_streams_for(profile->id, config_.instance);

    // Re-scale topic map while logging: suppressed profile duplicates are omitted
    rate_scale_ = scale_to_baud(map_, baud, profile->bandwidth, &Link::is_profile_stream, this);

    // Initialize dropouts tracking and drain stale samples
    std::array<std::uint8_t, max_message_size> discard{};
    for (std::size_t i = 0; i < profile_streams_.size() && i < last_missed_.size(); ++i) {
        while (profile_streams_[i]->next(now_us, discard) != 0) {
        }
        last_missed_[i] = profile_streams_[i]->missed();
        unsent_dropouts_[i] = 0;
    }

    // Answer with log_info
    session_.send_log_info(LogStartResult::Ok, config_.board_info, profile->bandwidth, sink_);

    // Request commander to restart the estimators (spec 0011, ticket #139)
    session_.request_restart_estimators(now_us);
}

void Link::on_log_stop()
{
    stop_logging();
}

void Link::stop_logging()
{
    if (!logging_) {
        return;
    }
    logging_ = false;
    active_profile_ = nullptr;
    profile_streams_ = {};
    const Link* owner = this;
    active_logger.compare_exchange_strong(owner, nullptr, std::memory_order_acq_rel);
    rate_scale_ = scale_to_baud(map_, transport_->baud_rate(), 0.0F);
}

bool Link::is_profile_topic(std::uint16_t topic_id, std::uint8_t instance) const
{
    if (active_profile_ == nullptr) {
        return false;
    }
    for (const auto& entry : active_profile_->entries) {
        if (entry.topic_id == topic_id && entry.instance == instance) {
            return true;
        }
    }
    return false;
}

void Link::send_topics(std::uint64_t now_us)
{
    // Held back, not drained, until the host knows every topic.
    if (handshake_state_ != HandshakeState::Connected || session_.describing()) {
        return;
    }
    std::array<std::uint8_t, max_message_size> message{};
    if (logging_ && active_profile_ != nullptr) {
        for (std::size_t i = 0; i < profile_streams_.size() && i < last_missed_.size(); ++i) {
            TopicStream* stream = profile_streams_[i];
            for (std::size_t size = stream->next(now_us, message); size != 0;
                 size = stream->next(now_us, message)) {
                if (!session_.send_topic(stream->topic_id(), stream->instance(), std::span(message).first(size),
                                         now_us, sink_)) {
                    ++unsent_dropouts_[i];
                }
            }
            const std::uint64_t missed = stream->missed();
            if (missed > last_missed_[i]) {
                const auto count = static_cast<std::uint16_t>(missed - last_missed_[i]);
                last_missed_[i] = missed;
                unsent_dropouts_[i] += count;
            }
            if (unsent_dropouts_[i] > 0) {
                if (session_.send_log_dropout(stream->topic_id(), stream->instance(), unsent_dropouts_[i], sink_)) {
                    unsent_dropouts_[i] = 0;
                }
            }
        }
    }
    for (TopicStream* stream : map_) {
        if (logging_ && is_profile_topic(stream->topic_id(), stream->instance())) {
            while (stream->next(now_us, message) != 0) {
            }
            continue;
        }
        for (std::size_t size = stream->next(now_us, message); size != 0; size = stream->next(now_us, message)) {
            session_.send_topic(stream->topic_id(), stream->instance(), std::span(message).first(size), now_us,
                                sink_);
        }
    }
}

bool Link::TransportSink::send(std::span<const std::uint8_t> wire)
{
    // Only whole frames go out; a partial one would cost the next frame too.
    if (transport_->write_space() < wire.size() || transport_->write(wire) != wire.size()) {
        ++drops;
        return false;
    }
    ++frames;
    bytes += static_cast<std::uint32_t>(wire.size());
    return true;
}

}  // namespace vk::link
