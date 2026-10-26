// SPDX-License-Identifier: GPL-3.0-or-later
// The control chain above actuator control (ADR-0013): a body velocity
// setpoint becomes a body motion demand, which the vehicle model allocates
// to a wheel speed target per actuator output. No bus, no FreeRTOS and no
// hardware, so it runs on the development host in tests; ControlChainTask
// runs it as a work item.
//
// Velocity control passes vx and vy to allocation as feedforward: the wheel
// speed loops of actuator control already close each wheel's speed. It
// limits how fast they change (VEL_ACC_MAX speeding up, VEL_DEC_MAX slowing
// down, #133), so a setpoint stepped to zero does not brake the wheels into
// a skid; a setpoint crossing zero slows to zero first. The limited
// setpoint starts from zero whenever the loops close.
// Angular rate control closes the yaw rate on the gyro (the attitude
// estimator's angular velocity, bias removed), so the vehicle turns at the
// commanded rate even when its wheels slip:
//
//   yaw rate demand = RATE_YAW_FF·target + RATE_YAW_P·error + integral
//
// The integral grows by RATE_YAW_I·error·dt, except while the last
// allocation limited the yaw rate axis (ArduPilot's AP_MotorsUGV limit
// flags, PX4's allocator feedback), so it does not wind up while the
// vehicle cannot follow. The error is taken against the target scaled as
// the last allocation scaled the demand: a saturated vehicle then turns at
// the rate its scaled-down speed allows, and keeps the commanded curvature,
// instead of the loop bending its path to chase the full rate. Without a yaw rate measurement the integral holds
// and the demand is the feedforward plus that integral. Not closing the
// loop (disarmed, the emergency stop, or the setpoint not driving) clears
// the integral and the limits. At rest, the limited setpoint zero on every
// axis, the integral is held at zero: what a skid left in it would
// otherwise turn a stopped vehicle (#133). A setpoint with a NaN field, or no vehicle
// model, leaves every output uncontrolled.
#pragma once

#include <vk/vehicle_model/vehicle_model.hpp>

#include <cstdint>
#include <optional>

namespace vk::control {

using vehicle_model::OutputTargets;
using vehicle_model::Twist;
using vehicle_model::TwistAxis;
using vehicle_model::VehicleModel;
using vehicle_model::WheelAllocation;

// The RATE_YAW_* parameters.
struct RateSettings {
    float p = 0.5F;   // RATE_YAW_P
    float i = 1.0F;   // RATE_YAW_I, 1/s
    float ff = 1.0F;  // RATE_YAW_FF
};

// The VEL_* parameters, in m/s²; 0 for no limit.
struct VelocityLimits {
    float acc_max = 0.0F;  // VEL_ACC_MAX
    float dec_max = 0.0F;  // VEL_DEC_MAX
};

// One cycle's result.
struct ChainResult {
    Twist demand{};               // the body motion demand, before allocation's scaling
    WheelAllocation allocation{};
};

class ControlChain {
public:
    // `model` may be nullptr: no vehicle model was selected at boot.
    explicit ControlChain(const VehicleModel* model) : model_(model) {}

    // One control cycle `dt_s` after the last: the wheel speed targets for
    // `setpoint`, none above `max_speed` [rad/s] (MOT_MAX_SPD). `closed` says
    // whether the loops run: armed, the emergency stop released and this
    // setpoint the one that drives. `limits` bounds how fast vx and vy
    // change while the loops close.
    ChainResult step(float dt_s, const Twist& setpoint, std::optional<float> measured_yaw_rate, bool closed,
                     const RateSettings& settings, float max_speed, OutputTargets targets,
                     const VelocityLimits& limits = {});

    [[nodiscard]] bool has_model() const { return model_ != nullptr; }
    [[nodiscard]] float yaw_integral() const { return yaw_integral_; }

private:
    void reset();

    const VehicleModel* model_;
    float yaw_integral_ = 0.0F;
    std::uint8_t limited_ = 0;  // TwistAxis bits of the last allocation
    float scale_ = 1.0F;        // the last allocation's scale
    float vx_ = 0.0F;           // the limited setpoint
    float vy_ = 0.0F;
};

}  // namespace vk::control
