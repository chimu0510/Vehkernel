// The motor-to-encoder mapping and direction bring-up case (ADR-0018): a
// state machine with no bus, no FreeRTOS and no hardware, run by
// ActuatorTask in test mode.
//
// For each output in turn it applies a fixed open-loop duty for the pulse
// time with every other output off, records how far every encoder moved,
// then rests so the wheel stops before the next output. MOTn_REV applies to
// the duty and MOTn_ENC_REV to the counts, so once they are set right every
// output passes. An encoder counts as moved when it turned at least
// min_turn of its wheel turn. For each output the result says whether its
// own encoder moved forwards (pass), backwards (reversed), not at all, or
// whether another encoder moved most; with that encoder and its sign.
#pragma once

#include <vk/actuators/actuator_control.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::actuators {

enum class BringupState : std::uint8_t {
    Idle = 0,
    Running = 1,
    Done = 2,
    Aborted = 3,
};

enum class BringupResult : std::uint8_t {
    Pass = 0,
    NotMoved = 1,      // no encoder moved
    Reversed = 2,      // its own encoder moved the wrong way
    OtherEncoder = 3,  // another encoder moved most
};

struct OutputResult {
    BringupResult result = BringupResult::NotMoved;
    std::uint8_t encoder = 0;  // the encoder that moved most, 1 to max_outputs, 0 for none
    std::int8_t sign = 0;
};

class BringupCase {
public:
    static constexpr float duty = 0.2F;              // within MOT_OUT_MAX
    static constexpr std::uint64_t pulse_us = 300'000;
    static constexpr std::uint64_t rest_us = 300'000;
    static constexpr float min_turn = 0.02F;         // of a wheel turn

    // Starts over `outputs` outputs; their encoders' counts now are the reference.
    void start(std::uint64_t now_us, std::size_t outputs, std::span<const std::uint16_t> counts);

    // One control cycle: returns each output's duty, as the driver gets it.
    std::array<float, max_outputs> step(std::uint64_t now_us, std::span<const std::uint16_t> counts,
                                        const Settings& settings);

    // Stops the case before it finished: every output off.
    void abort();

    [[nodiscard]] BringupState state() const { return state_; }
    [[nodiscard]] std::size_t outputs() const { return outputs_; }
    [[nodiscard]] const OutputResult& result(std::size_t output) const { return results_.at(output); }

private:
    void evaluate(const Settings& settings);

    BringupState state_ = BringupState::Idle;
    std::size_t outputs_ = 0;
    std::size_t current_ = 0;         // the output being pulsed
    std::uint64_t phase_start_us_ = 0;
    std::array<std::uint16_t, max_outputs> last_counts_{};
    std::array<std::int32_t, max_outputs> moved_{};  // counts during the current pulse, by encoder
    std::array<OutputResult, max_outputs> results_{};
};

}  // namespace vk::actuators
