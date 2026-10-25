// SPDX-License-Identifier: GPL-3.0-or-later
// Topic maps (ADR-0002): which topic instances leave through a Link, and how
// often.
//
// A topic map is a table of topic streams fixed at build time. Each outbound
// stream reads one topic instance and either sends every sample, for queued
// topics such as imu_sample, or sends the latest value at most at a given
// rate. An inbound stream publishes what the peer sends on its topic
// instance; the peer may send no other topic. Inbound setpoints pass setpoint
// arbitration first (ADR-0005).
//
// A Link on a transport with a baud rate scales its streams down when the map
// would need more than 80 % of the port (ADR-0002): every stream faster than
// 1 Hz by one common factor, never below 1 Hz, an every-sample stream
// becoming a rate-limited one; 1 Hz streams keep their rate.
#pragma once

#include <vk/core/bus.hpp>

#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::link {

// Send every message published, in order.
struct EverySample {};
inline constexpr EverySample every_sample{};

// Send the latest message at most `hz` times a second.
struct MaxRate {
    float hz;
};

// Receive the topic from the peer and publish it; a setpoint goes through
// setpoint arbitration first.
struct Inbound {
    bool setpoint = false;
};

class TopicStream {
public:
    TopicStream(const TopicStream&) = delete;
    TopicStream& operator=(const TopicStream&) = delete;
    TopicStream(TopicStream&&) = delete;
    TopicStream& operator=(TopicStream&&) = delete;

    // Encodes into `out` the next message due at `now_us`; returns its size,
    // 0 when none is due. Call until it returns 0.
    virtual std::size_t next(std::uint64_t now_us, std::span<std::uint8_t> out) = 0;

    // The rate the stream sends at before any scaling: its rate limit, or an
    // every-sample stream's nominal publish rate.
    [[nodiscard]] virtual float nominal_hz() const = 0;

    // The rate it sends at most now.
    [[nodiscard]] virtual float rate_hz() const = 0;

    // The encoded size of its messages.
    [[nodiscard]] virtual std::size_t message_size() const = 0;

    // Sends at most `hz` times a second from now on, an every-sample stream
    // only the newest sample of each interval.
    virtual void limit_rate(float hz) = 0;

    // Inbound streams only: whether the topic is a setpoint, and publishing a
    // message the peer sent, stamped with the time it arrived; false if it
    // does not decode.
    [[nodiscard]] virtual bool inbound() const = 0;
    [[nodiscard]] virtual bool setpoint() const = 0;
    virtual bool receive(std::span<const std::uint8_t> message, std::uint64_t now_us) = 0;

    [[nodiscard]] std::uint16_t topic_id() const { return topic_id_; }
    [[nodiscard]] std::uint8_t instance() const { return instance_; }

protected:
    TopicStream(std::uint16_t topic_id, std::uint8_t instance) : topic_id_(topic_id), instance_(instance) {}
    ~TopicStream() = default;

private:
    std::uint16_t topic_id_;
    std::uint8_t instance_;
};

template <class T>
class TopicStreamOf final : public TopicStream {
public:
    // `nominal_hz` is how often the topic is published, when the schema
    // leaves it to the board, as for imu_sample.
    explicit TopicStreamOf(EverySample /*unused*/, std::size_t instance = 0, float nominal_hz = T::nominal_rate_hz)
        : TopicStream(T::topic_id, static_cast<std::uint8_t>(instance)), subscription_(instance),
          publication_(instance), nominal_hz_(nominal_hz)
    {
    }

    explicit TopicStreamOf(MaxRate rate, std::size_t instance = 0)
        : TopicStream(T::topic_id, static_cast<std::uint8_t>(instance)), subscription_(instance),
          publication_(instance), interval_us_(static_cast<std::uint32_t>(1e6F / rate.hz)), nominal_hz_(rate.hz)
    {
    }

    explicit TopicStreamOf(Inbound direction, std::size_t instance = 0)
        : TopicStream(T::topic_id, static_cast<std::uint8_t>(instance)), subscription_(instance),
          publication_(instance), inbound_(true), setpoint_(direction.setpoint)
    {
    }

    [[nodiscard]] bool inbound() const override { return inbound_; }
    [[nodiscard]] bool setpoint() const override { return setpoint_; }

    bool receive(std::span<const std::uint8_t> message, std::uint64_t now_us) override
    {
        T decoded{};
        if (!inbound_ || !T::decode(message, decoded)) {
            return false;
        }
        decoded.timestamp = now_us;
        publication_.publish(decoded);
        return true;
    }

    [[nodiscard]] float nominal_hz() const override { return nominal_hz_; }
    [[nodiscard]] float rate_hz() const override
    {
        return interval_us_ == 0 ? nominal_hz_ : 1e6F / static_cast<float>(interval_us_);
    }
    [[nodiscard]] std::size_t message_size() const override { return T::encoded_size; }
    void limit_rate(float hz) override { interval_us_ = static_cast<std::uint32_t>(1e6F / hz); }

    std::size_t next(std::uint64_t now_us, std::span<std::uint8_t> out) override
    {
        T message{};
        if (inbound_) {
            return 0;
        }
        if (interval_us_ == 0) {
            return subscription_.copy(message) ? message.encode(out) : 0;
        }
        if (now_us < next_send_us_ || !subscription_.copy(message)) {
            return 0;
        }
        while (subscription_.copy(message)) {
            // A queued topic sent at a rate: only the newest sample leaves.
        }
        // Keep the rate on average, but never send a burst to catch up.
        next_send_us_ += interval_us_;
        if (next_send_us_ <= now_us) {
            next_send_us_ = now_us + interval_us_;
        }
        return message.encode(out);
    }

private:
    core::Subscription<T> subscription_;
    core::Publication<T> publication_;
    std::uint32_t interval_us_ = 0;  // 0 sends every sample
    float nominal_hz_ = 0.0F;  // 0 for an inbound stream, which the rate scaling leaves alone
    bool inbound_ = false;
    bool setpoint_ = false;
    std::uint64_t next_send_us_ = 0;
};

// A Link's topic map.
using TopicMap = std::span<TopicStream* const>;

// The share of a port's capacity a topic map may use.
inline constexpr float max_port_load = 0.8F;

// The bytes a second `map` needs at its current rates, each frame with its
// header, CRC, worst-case COBS overhead and delimiter.
float topic_map_load(TopicMap map);

// The bytes one run of a Link writes when every outbound stream of `map` is
// due at once, as the 1 Hz status streams are when the timing monitor
// publishes every work item's status together: what a transport's transmit
// buffer must hold on top of what is still draining.
std::size_t topic_map_burst(TopicMap map);

// The transmit buffer a Link's serial port needs for the default map's burst
// (topic_map_burst) with a frame of the run before still in it. A frame that
// does not fit is dropped whole and counted in link_status.
inline constexpr std::size_t serial_transmit_buffer_size = 2048;

// Scales `map` down to fit a port at `baud` bits a second (8N1, 10 bits a
// byte); returns the common factor in percent, 100 when nothing was scaled
// or the transport has no baud rate (0).
std::uint8_t scale_to_baud(TopicMap map, std::uint32_t baud);

// The default topic map: imu_sample instance 0 at every sample,
// vehicle_attitude, (in SITL) vehicle_attitude_groundtruth and
// vehicle_local_odometry_groundtruth, wheel_encoders, actuator_outputs,
// allocation_status, body_velocity_measurement instance 0 and
// vehicle_local_odometry at 50 Hz, vehicle_status and local_menu at 5 Hz, and
// parameter_status and every instance of work_item_status and link_status at
// 1 Hz; actuator_setpoint and body_velocity_setpoint come in, as setpoints.
// `imu_rate_hz` is imu_sample's publish rate, from the board's timing preset.
// Each Link, 0 to link_status's instances - 1, gets its own streams, so
// Links on ports at different baud rates scale their rates apart.
TopicMap default_topic_map(float imu_rate_hz, std::size_t link = 0);

}  // namespace vk::link
