// SPDX-License-Identifier: GPL-3.0-or-later
// Kinematics of a wheeled vehicle (ADR-0013, ADR-0014): the WHEELED vehicle
// model's math, one linear map between the body twist and the wheel speeds.
//
// The twist is in the body frame (x forward, y right, z down), so a positive
// yaw rate turns right. Each wheel sits at (x, y) from the body origin,
// rolls along body x with its radius, and has rollers at an angle to its
// axle (0 for a plain wheel, ±45° for mecanum). Its speed is one row of the
// wheel matrix J:
//
//   ω = (1/r)·(vx - y·yaw_rate + tan(γ)·(vy + x·yaw_rate))
//
// so `wheel speeds = J·twist` for control, and the least-squares
// pseudo-inverse `twist = J⁺·wheel speeds` for the measurement. An axis no
// wheel can move, such as vy for plain wheels, is left out of J⁺ and reads
// zero. The residual, the RMS of the wheels' rim speeds that the estimated
// twist does not explain, is zero when the wheels agree with a rigid body
// and grows with slip.
//
// On saturation the whole twist is scaled down by one factor, so the path
// keeps its curvature, and every axis with a demand is reported limited.
// Nothing allocates; J⁺ is computed once, when the wheels are set.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::estimation {

// A body motion in the body frame: the demand of the control chain or a
// measurement of the vehicle model.
struct Twist {
    float vx = 0.0F;        // [m/s] forward
    float vy = 0.0F;        // [m/s] right
    float yaw_rate = 0.0F;  // [rad/s] about z down, positive turning right
};

// The axes of a twist, as bits of a limited-axes report.
enum class TwistAxis : std::uint8_t { Vx = 1U << 0U, Vy = 1U << 1U, YawRate = 1U << 2U };

struct WheelGeometry {
    float x = 0.0F;             // [m] forward of the body origin
    float y = 0.0F;             // [m] right of the body origin
    float radius = 0.0F;        // [m]
    float roller_angle = 0.0F;  // [rad] 0 for a plain wheel
};

// The most wheels a model has: one per actuator output.
inline constexpr std::size_t max_wheels = 4;

// The result of an allocation.
struct WheelAllocation {
    float scale = 1.0F;        // the factor the twist was scaled by, 1 when it fits
    std::uint8_t limited = 0;  // TwistAxis bits of the axes the scaling limited
};

// The result of a measurement.
struct WheelMeasurement {
    Twist twist{};
    float residual = 0.0F;  // [m/s] RMS rim speed the twist does not explain
};

class WheelKinematics {
public:
    // No wheels: invalid until set.
    constexpr WheelKinematics() = default;

    // Builds J and J⁺. Invalid when there are no wheels or more than
    // max_wheels, a radius is not positive, the wheels cannot drive forward
    // and turn, as with a track of zero, or they cannot tell two axes apart
    // (J singular).
    explicit WheelKinematics(std::span<const WheelGeometry> wheels) { set(wheels); }

    void set(std::span<const WheelGeometry> wheels)
    {
        size_ = 0;
        valid_ = false;
        if (wheels.empty() || wheels.size() > max_wheels) {
            return;
        }
        size_ = wheels.size();
        for (std::size_t i = 0; i < size_; ++i) {
            const WheelGeometry& w = wheels[i];
            if (!(w.radius > 0.0F) || !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.roller_angle)) {
                return;
            }
            const float t = std::tan(w.roller_angle);
            j_[i] = {1.0F / w.radius, t / w.radius, (-w.y + t * w.x) / w.radius};
            radius_[i] = w.radius;
        }
        valid_ = invert();
    }

    [[nodiscard]] bool valid() const { return valid_; }
    [[nodiscard]] std::size_t size() const { return size_; }

    // Whether any wheel moves along `axis`; an axis that none does is
    // ignored by allocation and reads zero in a measurement.
    [[nodiscard]] bool moves(TwistAxis axis) const { return (active_ & static_cast<std::uint8_t>(axis)) != 0; }

    // One row of J: wheel `i`'s speed per unit of vx, vy and yaw rate.
    [[nodiscard]] const std::array<float, 3>& row(std::size_t i) const { return j_.at(i); }

    // wheel speeds = J·twist [rad/s], one per wheel.
    void wheel_speeds(const Twist& twist, std::span<float> speeds) const
    {
        for (std::size_t i = 0; i < size_ && i < speeds.size(); ++i) {
            speeds[i] = j_[i][0] * twist.vx + j_[i][1] * twist.vy + j_[i][2] * twist.yaw_rate;
        }
    }

    // The wheel speeds of `demand`, scaled down uniformly so none exceeds
    // `max_speed` [rad/s]. Axes no wheel moves are dropped from the demand.
    [[nodiscard]] WheelAllocation allocate(const Twist& demand, float max_speed, std::span<float> speeds) const
    {
        wheel_speeds(demand, speeds);
        float largest = 0.0F;
        for (std::size_t i = 0; i < size_ && i < speeds.size(); ++i) {
            largest = std::max(largest, std::fabs(speeds[i]));
        }
        const float limit = std::max(max_speed, 0.0F);
        WheelAllocation result{};
        if (largest <= limit) {
            return result;
        }
        result.scale = limit / largest;
        for (std::size_t i = 0; i < size_ && i < speeds.size(); ++i) {
            speeds[i] *= result.scale;
        }
        const std::array<float, 3> axes{demand.vx, demand.vy, demand.yaw_rate};
        for (std::size_t a = 0; a < axes.size(); ++a) {
            const auto bit = static_cast<std::uint8_t>(1U << a);
            if (axes[a] != 0.0F && (active_ & bit) != 0) {
                result.limited = static_cast<std::uint8_t>(result.limited | bit);
            }
        }
        return result;
    }

    // twist = J⁺·wheel speeds, and the residual of the fit.
    [[nodiscard]] WheelMeasurement measure(std::span<const float> speeds) const
    {
        WheelMeasurement result{};
        if (!valid_ || speeds.size() < size_) {
            return result;
        }
        std::array<float, 3> twist{};
        for (std::size_t a = 0; a < twist.size(); ++a) {
            for (std::size_t i = 0; i < size_; ++i) {
                twist[a] += pinv_[a][i] * speeds[i];
            }
        }
        result.twist = {.vx = twist[0], .vy = twist[1], .yaw_rate = twist[2]};
        std::array<float, max_wheels> fitted{};
        wheel_speeds(result.twist, fitted);
        float sum = 0.0F;
        for (std::size_t i = 0; i < size_; ++i) {
            const float rim = (speeds[i] - fitted[i]) * radius_[i];
            sum += rim * rim;
        }
        result.residual = std::sqrt(sum / static_cast<float>(size_));
        return result;
    }

private:
    // J⁺ = (JᵀJ)⁻¹Jᵀ over the axes some wheel moves. False when JᵀJ is
    // singular there.
    bool invert()
    {
        constexpr float tiny = 1e-9F;
        active_ = 0;
        std::array<std::size_t, 3> axes{};
        std::size_t k = 0;
        for (std::size_t a = 0; a < 3; ++a) {
            for (std::size_t i = 0; i < size_; ++i) {
                if (std::fabs(j_[i][a]) > tiny) {
                    active_ = static_cast<std::uint8_t>(active_ | (1U << a));
                    axes[k++] = a;
                    break;
                }
            }
        }
        constexpr auto drives =
            static_cast<std::uint8_t>(static_cast<unsigned>(TwistAxis::Vx) | static_cast<unsigned>(TwistAxis::YawRate));
        if ((active_ & drives) != drives || k > size_) {
            return false;
        }
        // [JᵀJ | Jᵀ] over the active axes, reduced by Gauss-Jordan with
        // partial pivoting to [I | J⁺].
        std::array<std::array<float, 3 + max_wheels>, 3> m{};
        for (std::size_t r = 0; r < k; ++r) {
            for (std::size_t c = 0; c < k; ++c) {
                for (std::size_t i = 0; i < size_; ++i) {
                    m[r][c] += j_[i][axes[r]] * j_[i][axes[c]];
                }
            }
            for (std::size_t i = 0; i < size_; ++i) {
                m[r][3 + i] = j_[i][axes[r]];
            }
        }
        float scale = 0.0F;
        for (std::size_t r = 0; r < k; ++r) {
            scale = std::max(scale, std::fabs(m[r][r]));
        }
        for (std::size_t c = 0; c < k; ++c) {
            std::size_t pivot = c;
            for (std::size_t r = c + 1; r < k; ++r) {
                if (std::fabs(m[r][c]) > std::fabs(m[pivot][c])) {
                    pivot = r;
                }
            }
            if (std::fabs(m[pivot][c]) <= 1e-6F * scale) {
                return false;
            }
            std::swap(m[c], m[pivot]);
            const float inv = 1.0F / m[c][c];
            for (float& v : m[c]) {
                v *= inv;
            }
            for (std::size_t r = 0; r < k; ++r) {
                if (r != c) {
                    const float f = m[r][c];
                    for (std::size_t col = 0; col < m[r].size(); ++col) {
                        m[r][col] -= f * m[c][col];
                    }
                }
            }
        }
        pinv_ = {};
        for (std::size_t r = 0; r < k; ++r) {
            for (std::size_t i = 0; i < size_; ++i) {
                pinv_[axes[r]][i] = m[r][3 + i];
            }
        }
        return true;
    }

    std::array<std::array<float, 3>, max_wheels> j_{};
    std::array<std::array<float, max_wheels>, 3> pinv_{};
    std::array<float, max_wheels> radius_{};
    std::size_t size_ = 0;
    std::uint8_t active_ = 0;  // TwistAxis bits some wheel moves along
    bool valid_ = false;
};

}  // namespace vk::estimation
