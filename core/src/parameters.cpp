#include <vk/core/parameters.hpp>

#include <vk/core/critical_section.hpp>

#include <algorithm>
#include <cmath>

namespace vk::core {

Parameters::Parameters(std::span<const ParamInfo> table, Clock clock) : table_(table), clock_(clock) {}

std::optional<std::size_t> Parameters::find(std::string_view name) const
{
    const auto it = std::ranges::lower_bound(table_, name, {}, &ParamInfo::name);
    if (it == table_.end() || it->name != name) {
        return std::nullopt;
    }
    return static_cast<std::size_t>(it - table_.begin());
}

ParamValue Parameters::get(std::size_t index) const
{
    const ParamInfo& info = table_[index];
    const CriticalSection lock;
    return info.type == ParamType::Float ? ParamValue::from_float(*info.float_value)
                                         : ParamValue::from_int(*info.int_value);
}

bool Parameters::in_range(const ParamInfo& info, ParamValue value)
{
    if (info.type == ParamType::Float) {
        const float f = value.as_float();
        return std::isfinite(f) && f >= info.min.as_float() && f <= info.max.as_float();
    }
    const std::int32_t i = value.as_int();
    if (!info.enum_values.empty()) {
        return std::ranges::find(info.enum_values, i) != info.enum_values.end();
    }
    if (info.bitmask != 0) {
        return (value.bits & ~info.bitmask) == 0;
    }
    return i >= info.min.as_int() && i <= info.max.as_int();
}

bool Parameters::armed()
{
    msg::ArmingState state{};
    if (arming_.copy(state)) {
        armed_ = state.armed;
    }
    return armed_;
}

ParamResult Parameters::set(std::size_t index, ParamValue value)
{
    const ParamInfo& info = table_[index];
    if (value.type != info.type) {
        return ParamResult::WrongType;
    }
    if (!in_range(info, value)) {
        return ParamResult::OutOfRange;
    }
    if ((info.flags & param_flags::locked_when_armed) != 0 && armed()) {
        return ParamResult::LockedWhileArmed;
    }
    std::uint32_t change = 0;
    {
        const CriticalSection lock;
        if (info.type == ParamType::Float) {
            if (std::bit_cast<std::uint32_t>(*info.float_value) == value.bits) {
                return ParamResult::Ok;
            }
            *info.float_value = value.as_float();
        } else {
            if (*info.int_value == value.as_int()) {
                return ParamResult::Ok;
            }
            *info.int_value = value.as_int();
        }
        change = ++changes_;
    }
    msg::ParameterUpdate update{};
    update.timestamp = clock_();
    update.name_hash = info.hash;
    update.change_count = change;
    update_.publish(update);
    return ParamResult::Ok;
}

ParamResult Parameters::set(std::string_view name, ParamValue value)
{
    const auto index = find(name);
    return index ? set(*index, value) : ParamResult::UnknownName;
}

}  // namespace vk::core
