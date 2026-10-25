// SPDX-License-Identifier: GPL-3.0-or-later
// Fuzz target for the session's frame handling (ADR-0002, ADR-0018): frames
// in any order and with any body reach Session::handle(), as the frame
// reader passes them on (session_input.hpp). The session has the build's
// parameter table and a fake commander on the bus. Topic data is decoded with the
// schema's decoder for its topic, as the Link does for an inbound topic.
//
// Checks, besides the sanitizers:
// - every frame the session sends is a single frame on the wire, no larger
//   than the largest, that the frame reader reads back with no sequence gap;
// - the session counts exactly the frames the sink took;
// - the negotiated version is 0 or the MCU's, and a connected session has
//   done its handshake.
#include "session_input.hpp"

#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/session.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/vehicle_command.hpp>
#include <vk/msg/vehicle_command_ack.hpp>
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

// Plays the commander on the bus: accepts or refuses each kind of request as
// the options byte says, on vehicle_command_ack.
class FakeControl final {
public:
    explicit FakeControl(std::uint8_t options) : options_(options)
    {
        vk::msg::VehicleCommand stale{};
        while (commands_.copy(stale)) {
        }
    }

    void process()
    {
        using vk::core::Command;
        vk::msg::VehicleCommand command{};
        while (commands_.copy(command)) {
            vk::msg::VehicleCommandAck ack{.source = command.source, .sequence = command.sequence,
                                           .command = command.command};
            switch (static_cast<Command>(command.command)) {
            case Command::Mode:
                if (refuses(session_options::refuse_mode)) {
                    ack.result = static_cast<std::uint8_t>(vk::core::ModeResult::Armed);
                } else if (command.argument <= static_cast<std::uint8_t>(vk::core::Mode::Test)) {
                    mode_ = static_cast<vk::core::Mode>(command.argument);
                } else {
                    ack.result = static_cast<std::uint8_t>(vk::core::ModeResult::UnknownMode);
                }
                break;
            case Command::Arm:
                if (command.argument != 0 && refuses(session_options::refuse_arming)) {
                    ack.result = static_cast<std::uint8_t>(vk::core::ArmingResult::WrongMode);
                } else {
                    armed_ = command.argument != 0;
                    publish_armed(armed_);
                }
                break;
            case Command::Bringup:
                if (refuses(session_options::refuse_bringup)) {
                    ack.result = static_cast<std::uint8_t>(vk::core::BringupStart::NotArmed);
                }
                break;
            case Command::Reboot:
                if (refuses(session_options::refuse_reboot)) {
                    ack.result = static_cast<std::uint8_t>(vk::core::RebootResult::Armed);
                }
                break;
            default:
                break;
            }
            ack.mode = static_cast<std::uint8_t>(mode_);
            ack.armed = armed_;
            ack.emergency_stop = command.command == static_cast<std::uint8_t>(Command::EmergencyStop) &&
                                 command.argument != 0;
            acks_.publish(ack);
        }
    }

private:
    [[nodiscard]] bool refuses(std::uint8_t option) const { return (options_ & option) != 0; }

    std::uint8_t options_;
    vk::core::Mode mode_ = vk::core::Mode::Standby;
    bool armed_ = false;
    vk::core::Subscription<vk::msg::VehicleCommand> commands_{0};
    vk::core::Publication<vk::msg::VehicleCommandAck> acks_;
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
    const vk::link::ControlAccess control_access{.enabled = (options & session_options::no_control) == 0,
                                                 .source = 0};
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
        control.process();
        session.send_answers(fake_now, sink);
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
