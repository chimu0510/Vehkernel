#include <vk/sim/motor.hpp>

#include <cmath>
#include <numbers>

namespace vk::sim {

Motor::Motor(const Config& config, Clock clock) : config_(config), clock_(clock), now_us_(clock()) {}

void Motor::drive(float duty)
{
    advance();
    duty_ = duty;
}

void Motor::set_powered(bool powered)
{
    advance();
    powered_ = powered;
}

void Motor::set_reversed(bool reversed)
{
    advance();
    reversed_ = reversed;
}

void Motor::set_encoder_reversed(bool reversed)
{
    encoder_reversed_ = reversed;
}

std::uint16_t Motor::count()
{
    advance();
    const double turns = (encoder_reversed_ ? -angle_ : angle_) / (2.0 * std::numbers::pi);
    const double counts = std::floor(turns * config_.counts_per_turn);
    // The counter wraps at 16 bits, as a timer's low half does.
    return static_cast<std::uint16_t>(static_cast<std::int64_t>(counts) & 0xFFFF);
}

float Motor::speed()
{
    advance();
    return speed_;
}

double Motor::angle()
{
    advance();
    return angle_;
}

void Motor::advance()
{
    const std::uint64_t now = clock_();
    if (now <= now_us_) {
        return;
    }
    const double dt = static_cast<double>(now - now_us_) * 1e-6;
    now_us_ = now;
    // The exact step of the first-order lag towards the speed the duty holds.
    const double duty = reversed_ ? -duty_ : duty_;
    const double target = powered_ ? duty * config_.max_speed : 0.0;
    const double decay = std::exp(-dt / config_.time_constant_s);
    const double start = speed_;
    const double next = target + (start - target) * decay;
    // The angle is the integral of the same curve.
    angle_ += target * dt + (start - target) * config_.time_constant_s * (1.0 - decay);
    speed_ = static_cast<float>(next);
}

}  // namespace vk::sim
