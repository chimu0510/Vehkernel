// SPDX-License-Identifier: GPL-3.0-or-later
// The parameter frames of a Link session (ADR-0002, ADR-0012), read and
// changed through the parameter runtime: param_get and param_set are answered
// at once with param_value or param_reject, and param_list_request starts a
// list of param_value frames that the session paces after the topic_info
// frames of its handshake.
#pragma once

#include <vk/core/parameter_storage.hpp>
#include <vk/core/parameters.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/frame_handler.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace vk::link {

// The parameters a session offers; without a runtime it offers none.
struct ParameterAccess {
    core::Parameters* parameters = nullptr;
    std::uint64_t set_hash = 0;                         // of the metadata the parameters come from
    const core::ParameterStorage* storage = nullptr;    // for the flags of param_value, if parameters are stored
};

class ParameterHandler {
public:
    explicit ParameterHandler(ParameterAccess access) : access_(access) {}

    // Handles param_list_request, param_get or param_set.
    void handle(FrameType type, BodyReader body, FrameSender& out);

    // The parameter set hash and count hello_reply carries.
    [[nodiscard]] std::uint64_t set_hash() const { return access_.set_hash; }
    [[nodiscard]] std::size_t count() const
    {
        return access_.parameters == nullptr ? 0 : access_.parameters->count();
    }

    // Whether a parameter list is still being sent.
    [[nodiscard]] bool listing() const { return list_next_ < count(); }

    // Sends the next param_value of the list; false if it was dropped, and
    // the same parameter goes again next time.
    bool send_next(FrameSender& out);

    // Ends a list in progress, for a new peer.
    void stop_list() { list_next_ = std::numeric_limits<std::size_t>::max(); }

private:
    void on_list(BodyReader body);
    void on_get(BodyReader body, FrameSender& out);
    void on_set(BodyReader body, FrameSender& out);
    bool send_value(std::size_t index, FrameSender& out);
    void send_reject(core::ParamResult reason, std::string_view name, FrameSender& out);

    ParameterAccess access_;
    // Next parameter of the list being sent; none at or past the count.
    std::size_t list_next_ = std::numeric_limits<std::size_t>::max();
};

}  // namespace vk::link
