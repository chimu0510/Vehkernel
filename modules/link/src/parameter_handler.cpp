// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/parameter_handler.hpp>

#include <array>
#include <optional>

namespace vk::link {
namespace {

// Reads a name: a length and that many characters.
std::string_view read_name(BodyReader& body)
{
    const auto bytes = body.bytes(body.u8());
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
}

void write_name(BodyWriter& out, std::string_view name)
{
    out.u8(static_cast<std::uint8_t>(name.size()));
    out.bytes({reinterpret_cast<const std::uint8_t*>(name.data()), name.size()});
}

}  // namespace

void ParameterHandler::handle(FrameType type, BodyReader body, FrameSender& out)
{
    switch (type) {
    case FrameType::ParamListRequest:
        on_list(body);
        break;
    case FrameType::ParamGet:
        on_get(body, out);
        break;
    case FrameType::ParamSet:
        on_set(body, out);
        break;
    default:
        break;
    }
}

bool ParameterHandler::send_next(FrameSender& out)
{
    if (!send_value(list_next_, out)) {
        return false;
    }
    ++list_next_;
    return true;
}

void ParameterHandler::on_list(BodyReader body)
{
    const std::uint16_t first = body.u16();
    if (body.ok()) {
        list_next_ = first;
    }
}

void ParameterHandler::on_get(BodyReader body, FrameSender& out)
{
    const std::string_view name = read_name(body);
    if (!body.ok()) {
        return;
    }
    const auto index = count() == 0 ? std::nullopt : access_.parameters->find(name);
    if (!index) {
        send_reject(core::ParamResult::UnknownName, name, out);
        return;
    }
    send_value(*index, out);
}

void ParameterHandler::on_set(BodyReader body, FrameSender& out)
{
    const std::string_view name = read_name(body);
    const std::uint8_t type = body.u8();
    const std::uint32_t bits = body.u32();
    if (!body.ok()) {
        return;
    }
    const auto index = count() == 0 ? std::nullopt : access_.parameters->find(name);
    if (!index) {
        send_reject(core::ParamResult::UnknownName, name, out);
        return;
    }
    const core::ParamValue value{.type = static_cast<core::ParamType>(type), .bits = bits};
    const core::ParamResult result = access_.parameters->set(*index, value);
    if (result != core::ParamResult::Ok) {
        send_reject(result, name, out);
        return;
    }
    send_value(*index, out);
}

bool ParameterHandler::send_value(std::size_t index, FrameSender& out)
{
    const core::ParamInfo& info = access_.parameters->info(index);
    const core::ParamValue value = access_.parameters->get(index);
    std::array<std::uint8_t, max_body_size> body{};
    BodyWriter writer(body);
    writer.u16(static_cast<std::uint16_t>(index));
    writer.u16(static_cast<std::uint16_t>(count()));
    writer.u8(static_cast<std::uint8_t>(value.type));
    writer.u32(value.bits);
    writer.u8(access_.storage == nullptr ? 0 : access_.storage->state(index));  // core::param_state
    write_name(writer, info.name);
    return writer.ok() && out.send(FrameType::ParamValue, writer.written());
}

void ParameterHandler::send_reject(core::ParamResult reason, std::string_view name, FrameSender& out)
{
    std::array<std::uint8_t, max_body_size> body{};
    BodyWriter writer(body);
    writer.u8(static_cast<std::uint8_t>(reason));
    write_name(writer, name);
    if (writer.ok()) {
        out.send(FrameType::ParamReject, writer.written());
    }
}

}  // namespace vk::link
