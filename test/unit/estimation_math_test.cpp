// SPDX-License-Identifier: GPL-3.0-or-later
// The estimation math library and its first models (ADR-0014), against
// closed forms and known angles.
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/estimation/models/quaternion_kinematics.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>

namespace {

using vk::estimation::AccelerometerModel;
using vk::estimation::Euler;
using vk::estimation::Quaternion;
using vk::estimation::QuaternionKinematics;
using vk::estimation::standard_gravity;
using vk::estimation::Vector3;

constexpr float pi = std::numbers::pi_v<float>;

float deg(float degrees)
{
    return degrees * pi / 180.0F;
}

void expect_near(const Vector3& actual, const Vector3& expected, float tolerance = 1e-5F)
{
    EXPECT_NEAR(actual.x, expected.x, tolerance);
    EXPECT_NEAR(actual.y, expected.y, tolerance);
    EXPECT_NEAR(actual.z, expected.z, tolerance);
}

TEST(Vector3, DotCrossAndNorm)
{
    const Vector3 a{1.0F, 2.0F, 3.0F};
    const Vector3 b{-2.0F, 0.5F, 4.0F};
    EXPECT_FLOAT_EQ(a.dot(b), 11.0F);
    expect_near(a.cross(b), {6.5F, -10.0F, 4.5F});
    EXPECT_FLOAT_EQ((Vector3{3.0F, 4.0F, 12.0F}).norm(), 13.0F);
    EXPECT_FLOAT_EQ(a.normalized().norm(), 1.0F);
    expect_near(Vector3{}.normalized(), {});
}

TEST(Quaternion, ProductComposesRotations)
{
    const Quaternion yaw90 = Quaternion::from_euler({.yaw = deg(90)});
    const Quaternion roll90 = Quaternion::from_euler({.roll = deg(90)});
    // Identity is neutral, and q times its conjugate is the identity.
    const Quaternion q = Quaternion::from_euler({.roll = 0.3F, .pitch = -0.2F, .yaw = 1.1F});
    EXPECT_NEAR((q * Quaternion::identity()).angle_to(q), 0.0F, 1e-6F);
    EXPECT_NEAR((q * q.conjugate()).angle_to(Quaternion::identity()), 0.0F, 1e-3F);
    // Body x forward, after yaw 90: east. Body y right, after roll 90: down.
    expect_near(yaw90.rotate({1.0F, 0.0F, 0.0F}), {0.0F, 1.0F, 0.0F});
    expect_near(roll90.rotate({0.0F, 1.0F, 0.0F}), {0.0F, 0.0F, 1.0F});
    // yaw90 * roll90: roll first, then yaw; body y goes down either way.
    expect_near((yaw90 * roll90).rotate({0.0F, 1.0F, 0.0F}), {0.0F, 0.0F, 1.0F});
    expect_near((yaw90 * roll90).rotate({1.0F, 0.0F, 0.0F}), {0.0F, 1.0F, 0.0F});
    // inverse_rotate undoes rotate.
    const Vector3 v{0.3F, -1.2F, 2.0F};
    expect_near(q.inverse_rotate(q.rotate(v)), v);
}

TEST(Quaternion, NormalizesAndKeepsUnitLength)
{
    const Quaternion q{2.0F, 0.0F, 0.0F, 0.0F};
    EXPECT_FLOAT_EQ(q.normalized().norm(), 1.0F);
    EXPECT_FLOAT_EQ(q.normalized().w, 1.0F);
    EXPECT_FLOAT_EQ(Quaternion::from_euler({.roll = 1.0F, .pitch = 0.5F, .yaw = -2.0F}).norm(), 1.0F);
}

TEST(Quaternion, EulerAnglesRoundTrip)
{
    const std::array<Euler, 5> cases{{
        {.roll = 0.0F, .pitch = 0.0F, .yaw = 0.0F},
        {.roll = deg(30), .pitch = deg(-20), .yaw = deg(170)},
        {.roll = deg(-179), .pitch = deg(45), .yaw = deg(-90)},
        {.roll = deg(10), .pitch = deg(89), .yaw = deg(5)},
        {.roll = deg(90), .pitch = deg(0), .yaw = deg(-45)},
    }};
    for (const Euler& e : cases) {
        const Euler back = Quaternion::from_euler(e).euler();
        EXPECT_NEAR(back.roll, e.roll, 1e-3F);
        EXPECT_NEAR(back.pitch, e.pitch, 1e-3F);
        EXPECT_NEAR(back.yaw, e.yaw, 1e-3F);
    }
}

TEST(Quaternion, GimbalLockKeepsTheRotation)
{
    // At pitch ±90° roll and yaw are not separable; the angles returned
    // must still describe the same rotation.
    for (const float pitch : {deg(90), deg(-90)}) {
        const Quaternion q = Quaternion::from_euler({.roll = deg(20), .pitch = pitch, .yaw = deg(50)});
        const Euler e = q.euler();
        EXPECT_FLOAT_EQ(e.roll, 0.0F);
        EXPECT_NEAR(e.pitch, pitch, 1e-6F);
        EXPECT_NEAR(Quaternion::from_euler(e).angle_to(q), 0.0F, 1e-3F);
    }
}

TEST(QuaternionKinematics, ConstantRateMatchesTheClosedForm)
{
    // 1000 steps of 5 ms at a constant rate about a tilted axis.
    const Vector3 omega = Vector3{0.3F, -0.5F, 0.8F};
    Quaternion q = Quaternion::from_euler({.roll = 0.2F, .pitch = 0.1F, .yaw = -0.4F});
    const Quaternion start = q;
    for (int i = 0; i < 1000; ++i) {
        q = QuaternionKinematics::predict(q, omega, 0.005F);
    }
    const Quaternion expected = start * Quaternion::from_rotation_vector(omega * 5.0F);
    EXPECT_NEAR(q.angle_to(expected), 0.0F, 1e-3F);
    EXPECT_NEAR(q.norm(), 1.0F, 1e-6F);
}

TEST(QuaternionKinematics, SmallRotationVectorsAreExact)
{
    const Quaternion q = Quaternion::from_rotation_vector({1e-8F, 0.0F, 0.0F});
    EXPECT_FLOAT_EQ(q.w, 1.0F);
    EXPECT_NEAR(q.x, 5e-9F, 1e-12F);
}

TEST(AccelerometerModel, PredictsTheSpecificForceAtKnownAttitudes)
{
    constexpr float g = standard_gravity;
    // Level: up is -z in the body frame.
    expect_near(AccelerometerModel::predict(Quaternion::identity()), {0.0F, 0.0F, -g}, 1e-4F);
    // Yaw does not change it.
    expect_near(AccelerometerModel::predict(Quaternion::from_euler({.yaw = 1.0F})), {0.0F, 0.0F, -g}, 1e-4F);
    // Nose up 30°: up leans forward.
    expect_near(AccelerometerModel::predict(Quaternion::from_euler({.pitch = deg(30)})),
                {g * 0.5F, 0.0F, -g * std::numbers::sqrt3_v<float> / 2.0F}, 1e-4F);
    // Right side down 30°: up leans left.
    expect_near(AccelerometerModel::predict(Quaternion::from_euler({.roll = deg(30)})),
                {0.0F, -g * 0.5F, -g * std::numbers::sqrt3_v<float> / 2.0F}, 1e-4F);
    // Upside down.
    expect_near(AccelerometerModel::predict(Quaternion::from_euler({.roll = pi})), {0.0F, 0.0F, g}, 1e-4F);
    EXPECT_NEAR(AccelerometerModel::predict_direction(Quaternion::from_euler({.roll = 0.4F})).norm(), 1.0F, 1e-6F);
}

TEST(AccelerometerModel, LevelsFromAMeasurementAtRest)
{
    for (const Euler& e : {Euler{.roll = deg(20), .pitch = deg(-35)}, Euler{.roll = deg(-170), .pitch = deg(10)},
                           Euler{.roll = 0.0F, .pitch = deg(80)}}) {
        const Euler level = AccelerometerModel::level_from(AccelerometerModel::predict(Quaternion::from_euler(e)));
        EXPECT_NEAR(level.roll, e.roll, 1e-4F);
        EXPECT_NEAR(level.pitch, e.pitch, 1e-4F);
    }
}

}  // namespace
