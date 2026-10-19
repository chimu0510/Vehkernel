// Topic maps (ADR-0002): which topic instances leave through a Link, and how
// often.
//
// A topic map is a table of topic streams fixed at build time, until the
// parameter system exists. Each stream reads one topic instance and either
// sends every sample, for queued topics such as imu_sample, or sends the
// latest value at most at a given rate.
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
    explicit TopicStreamOf(EverySample /*unused*/, std::size_t instance = 0)
        : TopicStream(T::topic_id, static_cast<std::uint8_t>(instance)), subscription_(instance)
    {
    }

    explicit TopicStreamOf(MaxRate rate, std::size_t instance = 0)
        : TopicStream(T::topic_id, static_cast<std::uint8_t>(instance)), subscription_(instance),
          interval_us_(static_cast<std::uint32_t>(1e6F / rate.hz))
    {
    }

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
    std::uint64_t next_send_us_ = 0;
};

// A Link's topic map.
using TopicMap = std::span<TopicStream* const>;

// The default topic map: imu_sample instance 0 at every sample,
// vehicle_attitude and (in SITL) vehicle_attitude_groundtruth at 50 Hz, and
// parameter_status and every instance of work_item_status and link_status at
// 1 Hz.
TopicMap default_topic_map();

}  // namespace vk::link
