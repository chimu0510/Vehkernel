// The parameter runtime (ADR-0012).
//
// Parameters are plain struct members in RAM, generated per parameter group
// from params.toml, that modules read directly. Every change goes through
// Parameters::set(): it checks the type, the range and the safety attributes,
// writes the member and publishes parameter_update. The parameter table, one
// ParamInfo per parameter sorted by name, comes from the parameter generator;
// a runtime can be built over any table, so tests bring their own.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/hal/clock.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/parameter_update.hpp>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace vk::core {

// FNV-1a, 32 bits, over a parameter name: the key of stored values (ADR-0012).
constexpr std::uint32_t param_hash(std::string_view name)
{
    std::uint32_t hash = 0x811C9DC5U;
    for (const char c : name) {
        hash ^= static_cast<std::uint8_t>(c);
        hash *= 0x01000193U;
    }
    return hash;
}

// Parameters are int32 or float32 (ADR-0012); booleans, enums and bitmasks are int32.
enum class ParamType : std::uint8_t {
    Int32 = 1,
    Float = 2,
};

// A parameter value with its type, as the 32 bits stored and sent.
struct ParamValue {
    ParamType type = ParamType::Int32;
    std::uint32_t bits = 0;

    static constexpr ParamValue from_int(std::int32_t value)
    {
        return {.type = ParamType::Int32, .bits = std::bit_cast<std::uint32_t>(value)};
    }
    static constexpr ParamValue from_float(float value)
    {
        return {.type = ParamType::Float, .bits = std::bit_cast<std::uint32_t>(value)};
    }

    [[nodiscard]] constexpr std::int32_t as_int() const { return std::bit_cast<std::int32_t>(bits); }
    [[nodiscard]] constexpr float as_float() const { return std::bit_cast<float>(bits); }

    friend constexpr bool operator==(const ParamValue&, const ParamValue&) = default;
};

namespace param_flags {
inline constexpr std::uint8_t reboot_required = 1U << 0U;    // takes effect at the next boot
inline constexpr std::uint8_t locked_when_armed = 1U << 1U;  // no changes while armed
}  // namespace param_flags

// One parameter of the table.
struct ParamInfo {
    std::string_view name;
    std::uint32_t hash;  // param_hash(name)
    ParamType type;
    std::uint8_t flags;  // param_flags
    ParamValue default_value;
    ParamValue min;
    ParamValue max;
    std::span<const std::int32_t> enum_values;  // enums only: the valid values
    std::uint32_t bitmask;                      // bitmasks only: the valid bits
    std::int32_t* int_value;                    // the struct member, for Int32
    float* float_value;                         // the struct member, for Float
};

// Why a change was refused; also what a Link reports back.
enum class ParamResult : std::uint8_t {
    Ok = 0,
    UnknownName = 1,
    WrongType = 2,
    OutOfRange = 3,
    LockedWhileArmed = 4,
};

class Parameters {
public:
    using Clock = std::uint64_t (*)() noexcept;

    // `table` must be sorted by name. `clock` is replaceable for tests.
    explicit Parameters(std::span<const ParamInfo> table, Clock clock = &hal::SystemClock::now_us);

    [[nodiscard]] std::size_t count() const { return table_.size(); }
    [[nodiscard]] const ParamInfo& info(std::size_t index) const { return table_[index]; }

    // The index of a parameter, or nothing if no parameter has that name.
    [[nodiscard]] std::optional<std::size_t> find(std::string_view name) const;
    // The same by the hash of the name, as stored entries carry it.
    [[nodiscard]] std::optional<std::size_t> find_hash(std::uint32_t hash) const;

    // The current value. `index` must be below count().
    [[nodiscard]] ParamValue get(std::size_t index) const;

    // Checks and applies a change. A change to a different value publishes
    // parameter_update; setting the current value again is accepted quietly.
    ParamResult set(std::size_t index, ParamValue value);
    ParamResult set(std::string_view name, ParamValue value);

    // Whether the vehicle is armed, from arming_state; disarmed until something publishes it.
    [[nodiscard]] bool armed();

private:
    [[nodiscard]] static bool in_range(const ParamInfo& info, ParamValue value);

    std::span<const ParamInfo> table_;
    Clock clock_;
    Publication<msg::ParameterUpdate> update_;
    Subscription<msg::ArmingState> arming_;
    bool armed_ = false;
    std::uint32_t changes_ = 0;
};

}  // namespace vk::core
