// Link frames of the custom protocol (ADR-0002).
//
// On the wire every frame is COBS-encoded and ends with a zero byte. Before
// encoding a frame holds, little-endian:
//
//   u8 version | u8 type | u16 sequence | body | u16 CRC-16/CCITT-FALSE
//
// The CRC covers everything before it. The body depends on the type:
//
//   topic_data          u16 topic ID, u8 instance, encoded message
//   hello               u8 count, u8 protocol versions[count]
//   hello_reply         u8 chosen version (0: no shared version), u16 topic count
//   topic_info          u16 topic ID, u8 instances, u64 schema hash, u8 name length, name
//   time_sync_request   u64 host send time (opaque to the MCU)
//   time_sync_reply     u64 host send time, u64 MCU receive time [us], u64 MCU send time [us]
#pragma once

#include <vk/link/cobs.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace vk::link {

// Frame layout version; peers negotiate it with hello (ADR-0002).
inline constexpr std::uint8_t protocol_version = 1;

enum class FrameType : std::uint8_t {
    TopicData = 0x01,
    Hello = 0x10,
    HelloReply = 0x11,
    TopicInfo = 0x12,
    TimeSyncRequest = 0x20,
    TimeSyncReply = 0x21,
};

constexpr std::optional<FrameType> to_frame_type(std::uint8_t raw)
{
    switch (raw) {
    case static_cast<std::uint8_t>(FrameType::TopicData):
        return FrameType::TopicData;
    case static_cast<std::uint8_t>(FrameType::Hello):
        return FrameType::Hello;
    case static_cast<std::uint8_t>(FrameType::HelloReply):
        return FrameType::HelloReply;
    case static_cast<std::uint8_t>(FrameType::TopicInfo):
        return FrameType::TopicInfo;
    case static_cast<std::uint8_t>(FrameType::TimeSyncRequest):
        return FrameType::TimeSyncRequest;
    case static_cast<std::uint8_t>(FrameType::TimeSyncReply):
        return FrameType::TimeSyncReply;
    default:
        return std::nullopt;
    }
}

inline constexpr std::size_t header_size = 4;
inline constexpr std::size_t crc_size = 2;
// Largest body: topic data with the largest message a frame carries (ADR-0006).
inline constexpr std::size_t max_message_size = 240;
inline constexpr std::size_t max_body_size = 3 + max_message_size;
inline constexpr std::size_t max_frame_size = header_size + max_body_size + crc_size;
// Largest frame on the wire, with COBS overhead and the delimiter.
inline constexpr std::size_t max_wire_size = cobs_max_encoded_size(max_frame_size) + 1;

// A received frame whose CRC checked; `body` points into the reader's buffer
// and is valid until the next call to the reader.
struct Frame {
    std::uint8_t version;
    FrameType type;
    std::uint16_t sequence;
    std::span<const std::uint8_t> body;
};

// Builds frames ready for the wire, numbering them in order.
class FrameWriter {
public:
    // Encodes one frame into `out`; returns its size on the wire, 0 if the
    // body is too large or `out` too small.
    std::size_t write(FrameType type, std::span<const std::uint8_t> body, std::span<std::uint8_t> out);

    [[nodiscard]] std::uint16_t next_sequence() const { return sequence_; }

private:
    std::uint16_t sequence_ = 0;
};

// Receive statistics of one FrameReader.
struct ReaderStats {
    std::uint32_t frames;       // frames with a valid CRC
    std::uint32_t crc_errors;   // frames dropped for a wrong CRC
    std::uint32_t malformed;    // invalid COBS, too short or too long
    std::uint32_t lost;         // frames missing from sequence gaps
};

// Splits a byte stream into frames. After any corruption it drops bytes up
// to the next delimiter and carries on.
class FrameReader {
public:
    // Consumes `bytes`, calling `on_frame(const Frame&)` for each valid frame.
    template <class OnFrame>
    void feed(std::span<const std::uint8_t> bytes, OnFrame&& on_frame)
    {
        for (const std::uint8_t byte : bytes) {
            if (byte != 0) {
                if (length_ < encoded_.size()) {
                    encoded_[length_++] = byte;
                } else {
                    overflow_ = true;
                }
                continue;
            }
            Frame frame{};
            if (finish(frame)) {
                on_frame(static_cast<const Frame&>(frame));
            }
        }
    }

    [[nodiscard]] const ReaderStats& stats() const { return stats_; }

private:
    // Decodes the bytes before a delimiter; true when they form a valid frame.
    bool finish(Frame& frame);

    std::array<std::uint8_t, max_wire_size> encoded_{};
    std::array<std::uint8_t, max_frame_size> decoded_{};
    std::size_t length_ = 0;
    bool overflow_ = false;
    bool have_sequence_ = false;
    std::uint16_t expected_sequence_ = 0;
    ReaderStats stats_{};
};

// Little-endian field access for frame bodies.
class BodyWriter {
public:
    explicit BodyWriter(std::span<std::uint8_t> out) : out_(out) {}

    void u8(std::uint8_t value) { put(value, 1); }
    void u16(std::uint16_t value) { put(value, 2); }
    void u64(std::uint64_t value) { put(value, 8); }
    void bytes(std::span<const std::uint8_t> data);

    // False if anything did not fit.
    [[nodiscard]] bool ok() const { return ok_; }
    [[nodiscard]] std::span<const std::uint8_t> written() const { return out_.first(at_); }

private:
    void put(std::uint64_t value, std::size_t size);

    std::span<std::uint8_t> out_;
    std::size_t at_ = 0;
    bool ok_ = true;
};

class BodyReader {
public:
    explicit BodyReader(std::span<const std::uint8_t> in) : in_(in) {}

    std::uint8_t u8() { return static_cast<std::uint8_t>(get(1)); }
    std::uint16_t u16() { return static_cast<std::uint16_t>(get(2)); }
    std::uint64_t u64() { return get(8); }
    std::span<const std::uint8_t> bytes(std::size_t size);
    std::span<const std::uint8_t> rest();

    // False if a read ran past the end.
    [[nodiscard]] bool ok() const { return ok_; }

private:
    std::uint64_t get(std::size_t size);

    std::span<const std::uint8_t> in_;
    std::size_t at_ = 0;
    bool ok_ = true;
};

}  // namespace vk::link
