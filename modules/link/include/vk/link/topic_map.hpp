// Topic maps (ADR-0002): which topic instances leave through a Link, and how
// often.
//
// A topic map is a table of topic streams fixed at build time. Each stream
// reads one topic instance and either sends every sample, for queued topics
// such as imu_sample, or sends the latest value at most at a given rate.
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
          nominal_hz_(nominal_hz)
    {
    }

    explicit TopicStreamOf(MaxRate rate, std::size_t instance = 0)
        : TopicStream(T::topic_id, static_cast<std::uint8_t>(instance)), subscription_(instance),
          interval_us_(static_cast<std::uint32_t>(1e6F / rate.hz)), nominal_hz_(rate.hz)
    {
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
    std::uint32_t interval_us_ = 0;  // 0 sends every sample
    float nominal_hz_ = 0.0F;
    std::uint64_t next_send_us_ = 0;
};

// A Link's topic map.
using TopicMap = std::span<TopicStream* const>;

// The share of a port's capacity a topic map may use.
inline constexpr float max_port_load = 0.8F;

// The bytes a second `map` needs at its current rates, each frame with its
// header, CRC, worst-case COBS overhead and delimiter.
float topic_map_load(TopicMap map);

// Scales `map` down to fit a port at `baud` bits a second (8N1, 10 bits a
// byte); returns the common factor in percent, 100 when nothing was scaled
// or the transport has no baud rate (0).
std::uint8_t scale_to_baud(TopicMap map, std::uint32_t baud);

// The default topic map: imu_sample instance 0 at every sample,
// vehicle_attitude and (in SITL) vehicle_attitude_groundtruth at 50 Hz,
// vehicle_status at 5 Hz, and parameter_status and every instance of
// work_item_status and link_status at 1 Hz.
// `imu_rate_hz` is imu_sample's publish rate, from the board's timing preset.
TopicMap default_topic_map(float imu_rate_hz);

}  // namespace vk::link
