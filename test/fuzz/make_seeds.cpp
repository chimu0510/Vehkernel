// SPDX-License-Identifier: GPL-3.0-or-later
// Writes the seed corpus of the Link fuzz targets at build time, so the
// seeds follow the schema and the protocol: one valid frame of each request
// the MCU answers, topic data for every schema topic, and a whole session.
//
//   vk_fuzz_seeds <directory>
//
// writes <directory>/cobs, <directory>/frame_reader and <directory>/session.
#include "session_input.hpp"

#include <vk/core/vehicle_control.hpp>
#include <vk/link/frame.hpp>
#include <vk/msg/topics.hpp>
#include <vk/params/params.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace {

using vk::link::FrameType;
using Bytes = std::vector<std::uint8_t>;

struct Seed {
    std::string name;
    FrameType type;
    Bytes body;
};

Bytes name_body(std::string_view name)
{
    Bytes body{static_cast<std::uint8_t>(name.size())};
    body.insert(body.end(), name.begin(), name.end());
    return body;
}

void put(Bytes& out, std::uint64_t value, std::size_t size)
{
    for (std::size_t i = 0; i < size; ++i) {
        out.push_back(static_cast<std::uint8_t>(value >> (8U * i)));
    }
}

template <class... Messages>
void add_topic_data(vk::msg::TopicList<Messages...> /*unused*/, std::vector<Seed>& seeds)
{
    const auto add = [&](auto message) {
        using Message = decltype(message);
        Bytes body;
        put(body, Message::topic_id, 2);
        body.push_back(0);  // instance
        std::array<std::uint8_t, Message::encoded_size> encoded{};
        const std::size_t size = message.encode(encoded);
        body.insert(body.end(), encoded.begin(), encoded.begin() + static_cast<std::ptrdiff_t>(size));
        seeds.push_back({"topic_data_" + std::string(Message::topic_name), FrameType::TopicData, body});
    };
    (add(Messages{}), ...);
}

std::vector<Seed> make_seeds()
{
    const vk::core::ParamInfo& param = vk::params::table().front();
    Bytes param_set = name_body(param.name);
    param_set.push_back(static_cast<std::uint8_t>(param.default_value.type));
    put(param_set, param.default_value.bits, 4);
    Bytes time_sync;
    put(time_sync, 0x0123456789ABCDEFULL, 8);

    std::vector<Seed> seeds{
        {"hello", FrameType::Hello, {1, vk::link::protocol_version}},
        {"hello_versions", FrameType::Hello, {3, 1, 2, vk::link::protocol_version}},
        {"time_sync_request", FrameType::TimeSyncRequest, time_sync},
        {"param_list_request", FrameType::ParamListRequest, {0, 0}},
        {"param_get", FrameType::ParamGet, name_body(param.name)},
        {"param_get_unknown", FrameType::ParamGet, name_body("NO_SUCH_PARAM")},
        {"param_set", FrameType::ParamSet, param_set},
        {"mode_request", FrameType::ModeRequest, {static_cast<std::uint8_t>(vk::core::Mode::Test)}},
        {"arming_request", FrameType::ArmingRequest, {1}},
        {"disarm_request", FrameType::ArmingRequest, {0}},
        {"estop_request", FrameType::EstopRequest, {1}},
        {"bringup_run", FrameType::BringupRun, {static_cast<std::uint8_t>(vk::core::BringupCase::Motors)}},
        {"reboot_request", FrameType::RebootRequest, {static_cast<std::uint8_t>(vk::core::RebootAction::Reboot)}},
        {"reboot_bootloader", FrameType::RebootRequest,
         {static_cast<std::uint8_t>(vk::core::RebootAction::Bootloader)}},
        {"log_start", FrameType::LogStart, {1, 0, 0, 0}},
        {"log_stop", FrameType::LogStop, {}},
    };
    add_topic_data(vk::msg::AllTopics{}, seeds);
    return seeds;
}

void write(const std::filesystem::path& path, std::span<const std::uint8_t> bytes)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!out) {
        std::fprintf(stderr, "vk_fuzz_seeds: cannot write %s\n", path.c_str());
        std::exit(1);
    }
}

// The frame on the wire, with its delimiter.
Bytes wire(vk::link::FrameWriter& writer, const Seed& seed)
{
    std::array<std::uint8_t, vk::link::max_wire_size> out{};
    const std::size_t size = writer.write(seed.type, seed.body, out);
    return {out.begin(), out.begin() + static_cast<std::ptrdiff_t>(size)};
}

void add_record(Bytes& input, const Seed& seed, std::uint8_t time_step)
{
    input.push_back(static_cast<std::uint8_t>(seed.type));
    input.push_back(time_step);
    input.push_back(static_cast<std::uint8_t>(seed.body.size()));
    input.insert(input.end(), seed.body.begin(), seed.body.end());
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::fprintf(stderr, "usage: vk_fuzz_seeds <directory>\n");
        return 2;
    }
    const std::filesystem::path root = argv[1];
    for (const char* target : {"cobs", "frame_reader", "session"}) {
        std::filesystem::create_directories(root / target);
    }
    const std::vector<Seed> seeds = make_seeds();
    const Seed& hello = seeds.front();

    vk::link::FrameWriter writer;
    Bytes stream{0xFF};  // frame_reader: chunks of 255 bytes
    Bytes session{0};    // session: no options
    for (const Seed& seed : seeds) {
        Bytes frame = wire(writer, seed);
        stream.insert(stream.end(), frame.begin(), frame.end());

        // frame_reader: the frame alone, read in one go.
        frame.insert(frame.begin(), 0xFF);
        write(root / "frame_reader" / seed.name, frame);
        // cobs: the bytes between the delimiters.
        write(root / "cobs" / seed.name, std::span(frame).subspan(1, frame.size() - 2));

        // session: the frame after a handshake, with each option.
        Bytes input{0};
        add_record(input, hello, 0);
        add_record(input, seed, 1);
        write(root / "session" / seed.name, input);
        add_record(session, seed, 1);
    }
    write(root / "frame_reader" / "whole_session", stream);
    // The whole session byte by byte, as from a slow serial port.
    stream.front() = 1;
    write(root / "frame_reader" / "whole_session_bytewise", stream);

    write(root / "session" / "whole_session", session);
    using namespace vk::fuzz::session_options;
    const std::array<std::uint8_t, 3> option_sets{
        static_cast<std::uint8_t>(refuse_mode | refuse_arming | refuse_bringup | refuse_reboot), drop_frames,
        static_cast<std::uint8_t>(no_control | no_parameters)};
    for (const std::uint8_t options : option_sets) {
        session.front() = options;
        write(root / "session" / ("whole_session_options_" + std::to_string(options)), session);
    }
    // The peer falls silent past the timeout, then asks for time sync.
    Bytes silent{0};
    add_record(silent, hello, 0);
    add_record(silent, {.name = "", .type = FrameType::TimeSyncRequest, .body = Bytes(8, 1)}, 200);
    write(root / "session" / "peer_silent", silent);
    return 0;
}
