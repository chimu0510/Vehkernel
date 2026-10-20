// A simulated DC motor with its gearbox, wheel and quadrature encoder, for
// SITL and unit tests (ADR-0018), as the ICM20948 and NOR Flash models are.
//
// The wheel speed follows the signed duty as a first-order lag: at full duty
// it settles at the maximum speed with the time constant. Without power the
// duty does nothing, as when a hardware switch cuts the driver's supply. The
// encoder counts `counts_per_turn` per wheel turn into a 16-bit counter that
// wraps, as a timer does. The model advances to the clock's time whenever it
// is driven or read.
#pragma once

#include <cstdint>

namespace vk::sim {

class Motor {
public:
    using Clock = std::uint64_t (*)() noexcept;

    struct Config {
        float max_speed = 50.0F;         // [rad/s] at full duty
        float time_constant_s = 0.1F;
        float counts_per_turn = 20000.0F;
    };

    Motor(const Config& config, Clock clock);

    // The duty from -1 to 1 from now on.
    void drive(float duty);

    // Whether the motor driver has power; without it the duty does nothing.
    void set_powered(bool powered);

    // The encoder counter now.
    [[nodiscard]] std::uint16_t count();

    // The wheel's true speed and angle now, for checks.
    [[nodiscard]] float speed();
    [[nodiscard]] double angle();

private:
    void advance();

    Config config_;
    Clock clock_;
    std::uint64_t now_us_;
    float duty_ = 0.0F;
    bool powered_ = true;
    float speed_ = 0.0F;
    double angle_ = 0.0;
};

}  // namespace vk::sim
