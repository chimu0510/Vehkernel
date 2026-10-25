// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/frame_handler.hpp>

#include <array>

namespace vk::link {

bool FrameSender::send(FrameType type, std::span<const std::uint8_t> body)
{
    std::array<std::uint8_t, max_wire_size> wire{};
    const std::size_t size = writer_->write(type, body, wire);
    if (size == 0) {
        return false;
    }
    if (!sink_->send(std::span(wire).first(size))) {
        writer_->retract();
        return false;
    }
    ++*frames_sent_;
    return true;
}

}  // namespace vk::link
