// Actuator output capabilities (ADR-0009): what actuator control drives and
// reads directly, without a driver in between.
//
// A motor output is an H-bridge driven by a PWM duty and two direction lines,
// paired with a quadrature encoder on the wheel. Which platform implements
// them is fixed at build time, but actuator control is a module and not a
// template, so it reaches them through these small interfaces, as Links reach
// their transports.
#pragma once

#include <cstdint>

namespace vk::hal {

// A PWM channel at a frequency set once at start. Channels on one timer
// share its frequency.
class PwmOutput {
public:
    PwmOutput() = default;
    PwmOutput(const PwmOutput&) = delete;
    PwmOutput& operator=(const PwmOutput&) = delete;
    PwmOutput(PwmOutput&&) = delete;
    PwmOutput& operator=(PwmOutput&&) = delete;

    // Before the first duty; false if the timer cannot reach `hz`, which then
    // keeps the board default.
    virtual bool set_frequency(std::uint32_t hz) = 0;

    // `duty` from 0 to 1. Each call also refreshes the platform's output
    // watchdog, which turns every output off when the calls stop.
    virtual void set_duty(float duty) = 0;

protected:
    ~PwmOutput() = default;
};

// The two direction lines of an H-bridge: IN1 high drives forward, IN2 high
// in reverse, both low lets the motor coast.
class DirectionLines {
public:
    DirectionLines() = default;
    DirectionLines(const DirectionLines&) = delete;
    DirectionLines& operator=(const DirectionLines&) = delete;
    DirectionLines(DirectionLines&&) = delete;
    DirectionLines& operator=(DirectionLines&&) = delete;

    virtual void set(bool in1, bool in2) = 0;

protected:
    ~DirectionLines() = default;
};

// A quadrature encoder's counter, read as 16 bits whatever the timer's width,
// so differences wrap the same way on every timer.
class QuadratureEncoder {
public:
    QuadratureEncoder() = default;
    QuadratureEncoder(const QuadratureEncoder&) = delete;
    QuadratureEncoder& operator=(const QuadratureEncoder&) = delete;
    QuadratureEncoder(QuadratureEncoder&&) = delete;
    QuadratureEncoder& operator=(QuadratureEncoder&&) = delete;

    [[nodiscard]] virtual std::uint16_t count() const = 0;

protected:
    ~QuadratureEncoder() = default;
};

// One actuator output.
struct MotorChannel {
    PwmOutput* pwm;
    DirectionLines* direction;
    QuadratureEncoder* encoder;
};

}  // namespace vk::hal
