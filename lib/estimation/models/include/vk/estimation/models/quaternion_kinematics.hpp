// SPDX-License-Identifier: GPL-3.0-or-later
// Process model: quaternion kinematics (ADR-0014). The attitude turns with
// the body angular rate; q_next = q ⊗ exp(ω·dt/2).
#pragma once

#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>

namespace vk::estimation {

struct QuaternionKinematics {
    // The attitude `dt` seconds later at the body rate `omega` [rad/s].
    [[nodiscard]] static Quaternion predict(const Quaternion& q, const Vector3& omega, float dt)
    {
        return q.integrated(omega, dt);
    }
};

}  // namespace vk::estimation
