// The bus: typed topics, publications and subscriptions (ADR-0008).
//
// Every topic is generated from the schema (ADR-0006). Its storage, one
// buffer per instance, is a static variable created at build time; nothing is
// allocated at runtime. A topic keeps its latest value, or the last
// T::queue_length samples when the schema declares @queue.
//
// Publish and read run in short critical sections. ISRs never publish; they
// notify the work item that does.
#pragma once

#include <vk/core/critical_section.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>

namespace vk::core {

// A snapshot of one topic instance for observability.
struct TopicStatus {
    std::string_view name;
    std::size_t instance;
    std::uint64_t publications;   // since boot
    std::uint64_t last_update_us; // timestamp of the latest message, 0 if none
    float rate_hz;                // average since the first publication, 0 if fewer than two
};

// The part of a topic that does not depend on its message type.
class TopicBase {
public:
    constexpr TopicBase(std::string_view name, std::size_t instance) : name_(name), instance_(instance) {}

    [[nodiscard]] TopicStatus status() const
    {
        const CriticalSection lock;
        float rate = 0.0F;
        if (generation_ > 1 && last_update_us_ > first_update_us_) {
            rate = static_cast<float>(static_cast<double>(generation_ - 1) * 1e6 /
                                      static_cast<double>(last_update_us_ - first_update_us_));
        }
        return {.name = name_,
                .instance = instance_,
                .publications = generation_,
                .last_update_us = last_update_us_,
                .rate_hz = rate};
    }

protected:
    // Called with the critical section held.
    void record_publication(std::uint64_t timestamp)
    {
        if (generation_ == 0) {
            first_update_us_ = timestamp;
        }
        last_update_us_ = timestamp;
        ++generation_;
    }

    std::uint64_t generation_ = 0;  // publications so far

private:
    std::string_view name_;
    std::size_t instance_;
    std::uint64_t first_update_us_ = 0;
    std::uint64_t last_update_us_ = 0;
};

template <class T>
class Topic : public TopicBase {
public:
    static constexpr std::size_t depth = T::queue_length;
    static_assert(depth >= 1);

    constexpr explicit Topic(std::size_t instance) : TopicBase(T::topic_name, instance) {}

    void publish(const T& message)
    {
        const CriticalSection lock;
        buffer_[generation_ % depth] = message;
        record_publication(message.timestamp);
    }

    [[nodiscard]] std::uint64_t generation() const
    {
        const CriticalSection lock;
        return generation_;
    }

    struct ReadResult {
        bool copied;
        std::uint64_t index;  // of the message copied
    };

    // Copies the first still-buffered message whose index (0 = first ever
    // published) is at least `from`. On a latest-value topic that is always
    // the newest message.
    ReadResult read(std::uint64_t from, T& out) const
    {
        const CriticalSection lock;
        if (from >= generation_) {
            return {.copied = false, .index = from};
        }
        const std::uint64_t oldest = generation_ > depth ? generation_ - depth : 0;
        const std::uint64_t index = from > oldest ? from : oldest;
        out = buffer_[index % depth];
        return {.copied = true, .index = index};
    }

private:
    std::array<T, depth> buffer_{};
};

namespace detail {

template <class T, std::size_t... I>
constexpr std::array<Topic<T>, sizeof...(I)> make_instances(std::index_sequence<I...> /*unused*/)
{
    return {Topic<T>(I)...};
}

}  // namespace detail

// The static storage of every instance of topic T.
template <class T>
inline constinit std::array<Topic<T>, T::instances> topic_instances =
    detail::make_instances<T>(std::make_index_sequence<T::instances>{});

template <class T>
Topic<T>& topic(std::size_t instance)
{
    if (instance >= T::instances) {
        fail("topic instance out of range");
    }
    return topic_instances<T>[instance];
}

// Publishes messages of type T on one instance.
template <class T>
class Publication {
public:
    explicit Publication(std::size_t instance = 0) : topic_(&topic<T>(instance)) {}

    void publish(const T& message) { topic_->publish(message); }

private:
    Topic<T>* topic_;
};

// Reads messages of type T from one instance.
//
// On a latest-value topic, copy() returns the newest message. On a queued
// topic it returns the oldest unread one, so calling it until it returns
// false drains the queue in order. Messages overwritten before they were read
// are counted by missed().
template <class T>
class Subscription {
public:
    explicit Subscription(std::size_t instance = 0) : topic_(&topic<T>(instance)) {}

    // Whether a message was published since the last copy().
    [[nodiscard]] bool updated() const { return topic_->generation() != next_; }

    // Copies the next message to read; returns false if there is none.
    bool copy(T& out)
    {
        const auto result = topic_->read(next_, out);
        if (!result.copied) {
            return false;
        }
        missed_ += result.index - next_;
        next_ = result.index + 1;
        return true;
    }

    // Messages published but never read, since this subscription was created.
    [[nodiscard]] std::uint64_t missed() const { return missed_; }

private:
    Topic<T>* topic_;
    std::uint64_t next_ = 0;  // index of the next message to read
    std::uint64_t missed_ = 0;
};

}  // namespace vk::core
