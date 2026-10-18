#include <vk/sim/motion.hpp>

#include <vk/estimation/models/accelerometer_model.hpp>

#include <array>
#include <cstddef>
#include <numbers>
#include <span>

namespace vk::sim {
namespace {

using estimation::Quaternion;
using estimation::Vector3;

constexpr float deg = std::numbers::pi_v<float> / 180.0F;

// Constant body rate from `start` to `end` seconds after boot.
struct Segment {
    float start;
    float end;
    Vector3 rate;  // [rad/s]
};

// Motion starts at 3 s, after the estimator's 1 s alignment and some margin.
constexpr std::array<Segment, 2> tilt{{
    {.start = 3.0F, .end = 4.0F, .rate = {20.0F * deg, 0.0F, 0.0F}},
    {.start = 6.0F, .end = 7.0F, .rate = {0.0F, 20.0F * deg, 0.0F}},
}};

constexpr std::array<Segment, 3> rotate{{
    {.start = 3.0F, .end = 5.0F, .rate = {45.0F * deg, 0.0F, 0.0F}},
    {.start = 5.5F, .end = 7.5F, .rate = {0.0F, 45.0F * deg, 0.0F}},
    {.start = 8.0F, .end = 10.0F, .rate = {0.0F, 0.0F, 45.0F * deg}},
}};

std::span<const Segment> segments(MotionProfile profile)
{
    switch (profile) {
    case MotionProfile::Tilt:
        return tilt;
    case MotionProfile::Rotate:
        return rotate;
    case MotionProfile::Still:
    case MotionProfile::Shake:
        break;
    }
    return {};
}

// Shake: 0.5 s bursts every 1.5 s from 3 s to 10 s.
Vector3 shake_acceleration(float t)
{
    if (t < 3.0F || t >= 10.0F) {
        return {};
    }
    const float phase = (t - 3.0F) - 1.5F * static_cast<float>(static_cast<int>((t - 3.0F) / 1.5F));
    return phase < 0.5F ? Vector3{6.0F, 0.0F, 0.0F} : Vector3{};
}

}  // namespace

std::optional<MotionProfile> motion_profile(std::string_view name)
{
    if (name == "still") {
        return MotionProfile::Still;
    }
    if (name == "tilt") {
        return MotionProfile::Tilt;
    }
    if (name == "rotate") {
        return MotionProfile::Rotate;
    }
    if (name == "shake") {
        return MotionProfile::Shake;
    }
    return std::nullopt;
}

MotionState motion_at(MotionProfile profile, std::uint64_t t_us)
{
    const float t = static_cast<float>(t_us) * 1e-6F;
    MotionState state;
    for (const Segment& segment : segments(profile)) {
        if (t <= segment.start) {
            break;
        }
        const float until = t < segment.end ? t : segment.end;
        state.attitude = state.attitude * Quaternion::from_rotation_vector(segment.rate * (until - segment.start));
        if (t < segment.end) {
            state.rate = segment.rate;
        }
    }
    if (profile == MotionProfile::Shake) {
        state.linear_acceleration = shake_acceleration(t);
    }
    state.attitude = state.attitude.normalized();
    return state;
}

void MotionImu::update(Icm20948& imu)
{
    const MotionState truth = motion_at(profile_, clock_());
    const Vector3 gyro = truth.rate + gyro_bias_;
    const Vector3 accel = estimation::AccelerometerModel::predict(truth.attitude) + truth.linear_acceleration;
    // Body (x forward, y right, z down) to sensor (x forward, y left, z up).
    imu.set_gyro({gyro.x, -gyro.y, -gyro.z});
    imu.set_accel({accel.x, -accel.y, -accel.z});
}

}  // namespace vk::sim
