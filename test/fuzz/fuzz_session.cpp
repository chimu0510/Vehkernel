// SPDX-License-Identifier: GPL-3.0-or-later
// Fuzz target for the session's frame handling (ADR-0002, ADR-0018): frames
// in any order and with any body reach Session::handle(), as the frame
// reader passes them on (session_input.hpp). The session has the build's
// parameter table and a fake vehicle control. Topic data is decoded with the
// schema's decoder for its topic, as the Link does for an inbound topic.
//
// Checks, besides the sanitizers:
// - every frame the session sends is a single frame on the wire, no larger
//   than the largest, that the frame reader reads back with no sequence gap;
// - the session counts exactly the frames the sink took;
// - the negotiated version is 0 or the MCU's, and a connected session has
//   done its handshake.
#include "session_input.hpp"

#include <vk/core/parameters.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/session.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/bringup_status.hpp>
#include <vk/msg/topics.hpp>
#include <vk/params/params.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <span>
#include <vector>

namespace {

using namespace vk::fuzz;
using vk::link::FrameType;

void check(bool condition)
{
    if (!condition) {
        std::abort();
    }
}

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

// Publishes the arming state the parameter runtime reads for locked parameters.
void publish_armed(bool armed)
{
    vk::msg::ArmingState state{};
    state.armed = armed;
    vk::core::Publication<vk::msg::ArmingState>{}.publish(state);
}

// Accepts or refuses each kind of request as the options byte says.
class FakeControl final : public vk::core::VehicleControl {
public:
    explicit FakeControl(std::uint8_t options) : options_(options) {}

    vk::core::ModeResult request_mode(vk::core::ControlSource /*source*/, vk::core::Mode mode) override
    {
        if (refuses(session_options::refuse_mode)) {
            return vk::core::ModeResult::Armed;
        }
        mode_ = mode;
        return vk::core::ModeResult::Ok;
    }
    vk::core::ArmingResult request_arming(vk::core::ControlSource /*source*/, bool arm) override
    {
        if (arm && refuses(session_options::refuse_arming)) {
            return vk::core::ArmingResult::WrongMode;
        }
        armed_ = arm;
        publish_armed(arm);
        return vk::core::ArmingResult::Ok;
    }
    bool request_emergency_stop(bool engage) override { return engage; }
    [[nodiscard]] vk::core::Mode mode() const override { return mode_; }
    [[nodiscard]] bool armed() const override { return armed_; }
    [[nodiscard]] std::uint32_t skipped_checks() const override { return 0; }
    [[nodiscard]] bool accepts_setpoint(vk::core::ControlSource /*source*/) const override { return armed_; }
    vk::core::BringupStart request_bringup(vk::core::ControlSource /*source*/, std::uint8_t /*bringup_case*/) override
    {
        return refuses(session_options::refuse_bringup) ? vk::core::BringupStart::NotArmed
                                                        : vk::core::BringupStart::Ok;
    }
    void source_lost(vk::core::ControlSource /*source*/) override {}
    vk::core::RebootResult request_reboot(vk::core::RebootAction /*action*/) override
    {
        return refuses(session_options::refuse_reboot) ? vk::core::RebootResult::Armed : vk::core::RebootResult::Ok;
    }

private:
    [[nodiscard]] bool refuses(std::uint8_t option) const { return (options_ & option) != 0; }

    std::uint8_t options_;
    vk::core::Mode mode_ = vk::core::Mode::Standby;
    bool armed_ = false;
};

// Reads back every frame the session sends, and drops some if told to.
class CheckingSink final : public vk::link::FrameSink {
public:
    explicit CheckingSink(bool drop_frames) : drop_frames_(drop_frames) {}

    bool send(std::span<const std::uint8_t> wire) override
    {
        check(!wire.empty() && wire.size() <= vk::link::max_wire_size);
        check(wire.back() == 0);
        check(std::ranges::count(wire, std::uint8_t{0}) == 1);
        if (drop_frames_ && ++offered_ % 3 == 0) {
            return false;
        }
        std::uint32_t frames = 0;
        reader_.feed(wire, [&](const vk::link::Frame& frame) {
            ++frames;
            check(frame.version == vk::link::protocol_version);
            check(frame.body.size() <= vk::link::max_body_size);
        });
        check(frames == 1);
        check(reader_.stats().lost == 0);
        ++taken;
        return true;
    }

    std::uint32_t taken = 0;

private:
    bool drop_frames_;
    std::uint32_t offered_ = 0;
    vk::link::FrameReader reader_;
};

template <class Message>
bool decode_as(std::span<const std::uint8_t> encoded)
{
    Message message{};
    if (!Message::decode(encoded, message)) {
        return false;
    }
    std::array<std::uint8_t, Message::encoded_size> again{};
    check(message.encode(again) == Message::encoded_size);
    return true;
}

// Decodes a topic data body with the decoder of its topic, as the Link does.
template <class... Messages>
void decode_topic_data(vk::msg::TopicList<Messages...> /*unused*/, std::span<const std::uint8_t> body)
{
    vk::link::BodyReader reader(body);
    const std::uint16_t topic_id = reader.u16();
    reader.u8();  // instance
    const auto message = reader.rest();
    if (reader.ok()) {
        (void)((Messages::topic_id == topic_id && decode_as<Messages>(message)) || ...);
    }
}

// Puts every parameter back to its default, so each input starts alike.
void reset_parameters()
{
    for (const vk::core::ParamInfo& info : vk::params::table()) {
        if (info.int_value != nullptr) {
            *info.int_value = info.default_value.as_int();
        }
        if (info.float_value != nullptr) {
            *info.float_value = info.default_value.as_float();
        }
    }
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    if (size == 0) {
        return 0;
    }
    const std::vector<std::uint8_t> input(data, data + size);
    const std::uint8_t options = input[0];

    fake_now = 1'000'000;
    reset_parameters();
    publish_armed(false);
    vk::core::Parameters parameters(vk::params::table(), &fake_clock);
    FakeControl control(options);
    CheckingSink sink((options & session_options::drop_frames) != 0);
    const vk::link::ParameterAccess parameter_access{
        .parameters = (options & session_options::no_parameters) != 0 ? nullptr : &parameters,
        .set_hash = vk::params::set_hash};
    const vk::link::ControlAccess control_access{
        .control = (options & session_options::no_control) != 0 ? nullptr : &control, .source = 0};
    vk::link::Session session(vk::link::schema_topics, vk::link::Session::default_peer_timeout_us, parameter_access,
                              control_access);

    std::uint16_t sequence = 0;
    std::span<const std::uint8_t> rest = std::span(input).subspan(1);
    while (rest.size() >= 3) {
        const std::uint8_t raw_type = rest[0];
        fake_now += rest[1] * session_time_step_us;
        const std::size_t length = std::min<std::size_t>(rest[2], rest.size() - 3);
        const auto body = rest.subspan(3, length);
        rest = rest.subspan(3 + length);

        // The frame reader passes on only known types.
        const auto type = vk::link::to_frame_type(raw_type);
        if (!type) {
            continue;
        }
        session.handle({.version = vk::link::protocol_version, .type = *type, .sequence = sequence++, .body = body},
                       fake_now, sink);
        if (*type == FrameType::TopicData && session.handshake_done()) {
            decode_topic_data(vk::msg::AllTopics{}, body);
        }
        if (*type == FrameType::BringupRun && body.size() >= 2) {
            vk::msg::BringupStatus status{};
            status.bringup_case = body[0];
            status.state = 2;
            status.outputs = body[1];
            session.send_bringup_result(status, sink);
        }
        session.send_pending(fake_now, sink, 4);
        const std::array<std::uint8_t, 4> message{1, 2, 3, 4};
        session.send_topic(0, 0, message, fake_now, sink);

        check(session.version() == 0 || session.version() == vk::link::protocol_version);
        check(!session.connected(fake_now) || session.handshake_done());
        check(session.frames_sent() == sink.taken);
    }
    return 0;
}
