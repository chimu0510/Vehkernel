#include <vk/link/frame.hpp>

#include <vk/link/crc16.hpp>

namespace vk::link {

std::size_t FrameWriter::write(FrameType type, std::span<const std::uint8_t> body, std::span<std::uint8_t> out)
{
    if (body.size() > max_body_size) {
        return 0;
    }
    std::array<std::uint8_t, max_frame_size> frame{};
    BodyWriter writer(frame);
    writer.u8(protocol_version);
    writer.u8(static_cast<std::uint8_t>(type));
    writer.u16(sequence_);
    writer.bytes(body);
    writer.u16(crc16(writer.written()));
    const auto plain = writer.written();
    if (!writer.ok() || out.size() < cobs_max_encoded_size(plain.size()) + 1) {
        return 0;
    }
    const std::size_t size = cobs_encode(plain, out);
    out[size] = 0;
    ++sequence_;
    return size + 1;
}

bool FrameReader::finish(Frame& frame)
{
    const std::size_t length = length_;
    const bool overflow = overflow_;
    length_ = 0;
    overflow_ = false;
    if (length == 0) {
        return false;  // Back-to-back delimiters carry nothing.
    }
    if (overflow) {
        ++stats_.malformed;
        return false;
    }
    const std::size_t size = cobs_decode(std::span(encoded_).first(length), decoded_);
    if (size < header_size + crc_size) {
        ++stats_.malformed;
        return false;
    }
    const auto plain = std::span<const std::uint8_t>(decoded_).first(size);
    BodyReader trailer(plain.last(crc_size));
    if (crc16(plain.first(size - crc_size)) != trailer.u16()) {
        ++stats_.crc_errors;
        return false;
    }
    BodyReader header(plain);
    const std::uint8_t version = header.u8();
    const auto type = to_frame_type(header.u8());
    if (!type.has_value()) {
        ++stats_.malformed;
        return false;
    }
    frame.version = version;
    frame.type = *type;
    frame.sequence = header.u16();
    frame.body = plain.subspan(header_size, size - header_size - crc_size);

    if (have_sequence_) {
        const auto gap = static_cast<std::uint16_t>(frame.sequence - expected_sequence_);
        // A jump backwards or very far is a restarted peer, not a loss.
        if (gap < 0x8000U) {
            stats_.lost += gap;
        }
    }
    have_sequence_ = true;
    expected_sequence_ = static_cast<std::uint16_t>(frame.sequence + 1);
    ++stats_.frames;
    return true;
}

void BodyWriter::put(std::uint64_t value, std::size_t size)
{
    if (at_ + size > out_.size()) {
        ok_ = false;
        return;
    }
    for (std::size_t i = 0; i < size; ++i) {
        out_[at_++] = static_cast<std::uint8_t>(value >> (8U * i));
    }
}

void BodyWriter::bytes(std::span<const std::uint8_t> data)
{
    if (at_ + data.size() > out_.size()) {
        ok_ = false;
        return;
    }
    for (const std::uint8_t byte : data) {
        out_[at_++] = byte;
    }
}

std::uint64_t BodyReader::get(std::size_t size)
{
    if (at_ + size > in_.size()) {
        ok_ = false;
        at_ = in_.size();
        return 0;
    }
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {
        value |= static_cast<std::uint64_t>(in_[at_++]) << (8U * i);
    }
    return value;
}

std::span<const std::uint8_t> BodyReader::bytes(std::size_t size)
{
    if (at_ + size > in_.size()) {
        ok_ = false;
        at_ = in_.size();
        return {};
    }
    const auto data = in_.subspan(at_, size);
    at_ += size;
    return data;
}

std::span<const std::uint8_t> BodyReader::rest()
{
    const auto data = in_.subspan(at_);
    at_ = in_.size();
    return data;
}

}  // namespace vk::link
