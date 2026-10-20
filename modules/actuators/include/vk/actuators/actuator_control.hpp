// Actuator control (ADR-0013): the last layer of the control chain, a wheel
// speed PI per motor output on its encoder. No bus, no FreeRTOS and no
// hardware, so it runs on the development host in tests; ActuatorTask runs
// it as a work item.
//
// Each step reads every encoder as a 16-bit count and takes the difference
// to the last one as a 16-bit number, so 16- and 32-bit timers wrap the same
// way. The target is clamped to MOT_MAX_SPD; the duty is
// MOT_SPD_FF·target + MOT_SPD_P·error + integral, clamped to MOT_OUT_MAX,
// and the integral does not grow towards a side where the duty is clamped.
// MOTn_ENC_REV flips the counts, MOTn_REV the duty the driver gets. A NaN
// target, or outputs not enabled (disarmed or the emergency stop engaged),
// give no duty and clear the integral, so a wheel never jumps when control
// resumes. A zero target is held at zero speed; there is no brake mode.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

namespace vk::actuators {

// The most outputs any board has: the length of the actuator topics' arrays.
inline constexpr std::size_t max_outputs = 4;

struct OutputSettings {
    bool reversed = false;          // MOTn_REV
    bool encoder_reversed = false;  // MOTn_ENC_REV
    float counts_per_turn = 20000.0F;  // MOTn_CPR
};

struct LoopSettings {
    float p = 0.01F;          // MOT_SPD_P
    float i = 0.05F;          // MOT_SPD_I
    float ff = 0.02F;         // MOT_SPD_FF
    float max_speed = 30.0F;  // MOT_MAX_SPD
    float out_max = 0.3F;     // MOT_OUT_MAX
};

struct Settings {
    std::array<OutputSettings, max_outputs> outputs{};
    LoopSettings loop{};
};

// One output after a step.
struct Output {
    double angle = 0.0;  // [rad] accumulated since the first step
    float speed = 0.0F;  // [rad/s]
    float target = std::numeric_limits<float>::quiet_NaN();  // [rad/s] after clamping; NaN when uncontrolled
    float duty = 0.0F;   // -1 to 1, as the driver gets it
};

class ActuatorControl {
public:
    explicit ActuatorControl(std::size_t outputs);

    // One control cycle `dt_s` after the last: each output's encoder count,
    // its speed target (NaN leaves it uncontrolled), and whether outputs may
    // drive at all.
    void step(float dt_s, std::span<const std::uint16_t> counts, std::span<const float> targets, bool enabled,
              const Settings& settings);

    [[nodiscard]] const Output& output(std::size_t index) const { return wheels_.at(index).out; }
    [[nodiscard]] std::size_t size() const { return size_; }

private:
    struct Wheel {
        Output out;
        float integral = 0.0F;
        std::uint16_t last_count = 0;
        bool primed = false;
    };

    std::array<Wheel, max_outputs> wheels_{};
    std::size_t size_;
};

}  // namespace vk::actuators
