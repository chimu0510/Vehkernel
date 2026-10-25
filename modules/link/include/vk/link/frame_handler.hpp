// SPDX-License-Identifier: GPL-3.0-or-later
// What the handlers of a Link session (ADR-0002) answer through: the sink a
// session writes to and the sender that numbers and counts its frames, and
// the interface of a handler a session is given from outside, for frames
// only some builds accept.
#pragma once

#include <vk/link/frame.hpp>

#include <cstdint>
#include <span>

namespace vk::link {

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

// Sends frames of one session to a sink: numbered by the session's writer,
// and counted in its sent frames.
class FrameSender {
public:
    FrameSender(FrameWriter& writer, FrameSink& sink, std::uint32_t& frames_sent)
        : writer_(&writer), sink_(&sink), frames_sent_(&frames_sent)
    {
    }

    // Sends one frame; false if it did not fit or the sink dropped it.
    bool send(FrameType type, std::span<const std::uint8_t> body);

private:
    FrameWriter* writer_;
    FrameSink* sink_;
    std::uint32_t* frames_sent_;
};

// Handles frames a session does not handle itself, once the handshake has
// succeeded. Through this interface the session calls a handler without
// linking it, so a build that does not give one never carries its code.
class FrameHandler {
public:
    FrameHandler() = default;
    virtual ~FrameHandler() = default;
    FrameHandler(const FrameHandler&) = delete;
    FrameHandler& operator=(const FrameHandler&) = delete;
    FrameHandler(FrameHandler&&) = delete;
    FrameHandler& operator=(FrameHandler&&) = delete;

    // Handles one frame of a type the handler is given.
    virtual void handle(FrameType type, BodyReader body, FrameSender& out) = 0;
};

}  // namespace vk::link
