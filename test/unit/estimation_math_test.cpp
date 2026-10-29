// SPDX-License-Identifier: GPL-3.0-or-later
// The estimation math library and its first models (ADR-0014), against
// closed forms and known angles.
#include <vk/estimation/math/matrix.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>
#include <vk/estimation/models/accelerometer_model.hpp>
#include <vk/estimation/models/magnetometer_model.hpp>
#include <vk/estimation/models/quaternion_kinematics.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>

namespace {

using vk::estimation::AccelerometerModel;
using vk::estimation::Euler;
using vk::estimation::MagnetometerModel;
using vk::estimation::Matrix;
using vk::estimation::Matrix3;
using vk::estimation::Matrix6;
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

TEST(Matrix, ArithmeticTransposeSymmetricAndBlocks)
{
    const auto a = Matrix<2, 3>::from_rows({{{{1.0F, 2.0F, -1.0F}}, {{0.5F, -3.0F, 4.0F}}}});
    const auto b = Matrix<2, 3>::from_rows({{{{-2.0F, 1.0F, 3.0F}}, {{1.5F, 2.0F, -1.0F}}}});
    const Matrix<2, 3> sum = a + b;
    EXPECT_FLOAT_EQ(sum(0, 0), -1.0F);
    EXPECT_FLOAT_EQ(sum(0, 2), 2.0F);
    EXPECT_FLOAT_EQ(sum(1, 1), -1.0F);

    const Matrix<2, 3> diff = a - b;
    EXPECT_FLOAT_EQ(diff(0, 0), 3.0F);
    EXPECT_FLOAT_EQ(diff(1, 2), 5.0F);

    const Matrix<2, 3> scaled = 2.0F * a;
    EXPECT_FLOAT_EQ(scaled(0, 1), 4.0F);
    EXPECT_FLOAT_EQ((a * -2.0F)(0, 1), (-2.0F * a)(0, 1));
    EXPECT_FLOAT_EQ((-a)(1, 2), -4.0F);

    const Matrix<3, 2> bt = b.transposed();
    const Matrix<2, 2> prod = a * bt;
    // Row 0 of a dot row 0 of b: 1*(-2) + 2*1 + (-1)*3 = -3.
    EXPECT_FLOAT_EQ(prod(0, 0), -3.0F);
    // Row 1 of a dot row 1 of b: 0.5*1.5 + (-3)*2 + 4*(-1) = -9.25.
    EXPECT_FLOAT_EQ(prod(1, 1), -9.25F);

    Matrix3 m = Matrix3::identity();
    m(0, 2) = 4.0F;
    m(2, 0) = 2.0F;
    const Matrix3 sym = m.symmetric();
    EXPECT_FLOAT_EQ(sym(0, 2), 3.0F);
    EXPECT_FLOAT_EQ(sym(2, 0), 3.0F);
    EXPECT_FLOAT_EQ(sym(1, 1), 1.0F);

    Matrix6 big = Matrix6::zero();
    big.set_block(0, 3, sym);
    const Matrix3 extracted = big.block<3, 3>(0, 3);
    EXPECT_FLOAT_EQ(extracted(0, 2), 3.0F);
    EXPECT_FLOAT_EQ(extracted(1, 1), 1.0F);
}

TEST(Matrix, SkewMatchesCrossProductAndInvertsUpTo3x3)
{
    const Vector3 u{1.5F, -2.0F, 0.75F};
    const Vector3 v{-0.5F, 3.0F, 2.0F};
    const Vector3 from_skew = (Matrix3::skew(u) * Matrix<3, 1>::from_vector(v)).vector();
    expect_near(from_skew, u.cross(v));

    const Quaternion q = Quaternion::from_euler({.roll = 0.3F, .pitch = -0.2F, .yaw = 1.1F});
    const Matrix3 r_mat = Matrix3::rotation(q);
    expect_near((r_mat * Matrix<3, 1>::from_vector(v)).vector(), q.rotate(v));
    expect_near((r_mat.transposed() * Matrix<3, 1>::from_vector(v)).vector(), q.inverse_rotate(v));

    // 1x1 inverse.
    const auto inv1 = Matrix<1, 1>::from_rows({{{{4.0F}}}}).inverted();
    ASSERT_TRUE(inv1.has_value());
    EXPECT_FLOAT_EQ(inv1.value_or(Matrix<1, 1>{})(0, 0), 0.25F);
    EXPECT_FALSE((Matrix<1, 1>::zero().inverted().has_value()));

    // 2x2 inverse.
    const auto m2 = Matrix<2, 2>::from_rows({{{{4.0F, 7.0F}}, {{2.0F, 6.0F}}}});
    const auto inv2 = m2.inverted();
    ASSERT_TRUE(inv2.has_value());
    const Matrix<2, 2> id2 = m2 * inv2.value_or(Matrix<2, 2>{});
    EXPECT_NEAR(id2(0, 0), 1.0F, 1e-6F);
    EXPECT_NEAR(id2(0, 1), 0.0F, 1e-6F);
    EXPECT_NEAR(id2(1, 0), 0.0F, 1e-6F);
    EXPECT_NEAR(id2(1, 1), 1.0F, 1e-6F);
    const auto singular2 = Matrix<2, 2>::from_rows({{{{1.0F, 2.0F}}, {{2.0F, 4.0F}}}});
    EXPECT_FALSE(singular2.inverted().has_value());

    // 3x3 inverse: A * A^-1 = I.
    const auto m3 = Matrix3::from_rows({{{{2.0F, -1.0F, 0.5F}}, {{-1.0F, 3.0F, 1.0F}}, {{0.5F, 1.0F, 4.0F}}}});
    const auto inv3 = m3.inverted();
    ASSERT_TRUE(inv3.has_value());
    const Matrix3 id3 = m3 * inv3.value_or(Matrix3{});
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(id3(r, c), r == c ? 1.0F : 0.0F, 1e-5F);
        }
    }
    const auto singular3 = Matrix3::from_rows({{{{1.0F, 2.0F, 3.0F}}, {{2.0F, 4.0F, 6.0F}}, {{0.0F, 1.0F, 1.0F}}}});
    EXPECT_FALSE(singular3.inverted().has_value());
}

TEST(AccelerometerModel, GravityMeasurementAndRejection)
{
    const Quaternion q = Quaternion::from_euler({.roll = deg(15), .pitch = deg(-10), .yaw = deg(40)});
    const Vector3 at_rest = AccelerometerModel::predict(q);
    const auto exact = AccelerometerModel::gravity_measurement(q, at_rest, 0.5F);
    expect_near(exact.innovation, {});
    EXPECT_NEAR(exact.variance, (0.5F / standard_gravity) * (0.5F / standard_gravity), 1e-7F);

    // Small attitude perturbation delta_theta in the NED frame gives innovation ~ H * [delta_theta; 0].
    const Vector3 delta_theta{0.01F, -0.015F, 0.02F};
    const Quaternion q_true = Quaternion::from_rotation_vector(delta_theta) * q;
    const auto perturbed = AccelerometerModel::gravity_measurement(q, AccelerometerModel::predict(q_true), 0.5F);
    Matrix<6, 1> dx{};
    dx.set_block(0, 0, Matrix<3, 1>::from_vector(delta_theta));
    const Vector3 linear = (perturbed.jacobian * dx).vector();
    expect_near(perturbed.innovation, linear, 5e-4F);

    // Magnitude far from g is rejected before the gate.
    EXPECT_TRUE(AccelerometerModel::measure_gravity(q, at_rest, 0.5F, 0.1F).has_value());
    EXPECT_FALSE(AccelerometerModel::measure_gravity(q, at_rest * 1.2F, 0.5F, 0.1F).has_value());
}

TEST(MagnetometerModel, LevelsFieldAppliesDeclinationAndWrapsAcrossPi)
{
    const Vector3 earth_ned{30.0F, 0.0F, 30.0F};
    const Quaternion tilted = Quaternion::from_euler({.roll = deg(25), .pitch = deg(-20), .yaw = deg(65)});
    const Vector3 body_field = MagnetometerModel::predict(tilted, earth_ned);

    // Heading recovered despite roll, pitch and a strong vertical field component.
    EXPECT_NEAR(MagnetometerModel::heading_from(tilted, body_field), deg(65), 1e-4F);
    EXPECT_NEAR(MagnetometerModel::heading_from(tilted, body_field, deg(10)), deg(75), 1e-4F);

    // Jacobian touches the yaw error (state 2) only; roll, pitch and bias entries are zero.
    const auto m = MagnetometerModel::heading_measurement(tilted, body_field, 0.1F);
    EXPECT_NEAR(m.innovation, 0.0F, 1e-5F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 0), 0.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 1), 0.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 2), 1.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 3), 0.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 4), 0.0F);
    EXPECT_FLOAT_EQ(m.jacobian(0, 5), 0.0F);
    EXPECT_FLOAT_EQ(m.variance, 0.01F);

    // Wrapping across +-pi: true yaw -170 deg vs estimate +170 deg is +20 deg, and the reverse is -20 deg.
    const Quaternion est_pos = Quaternion::from_euler({.yaw = deg(170)});
    const Quaternion true_neg = Quaternion::from_euler({.yaw = deg(-170)});
    const auto wrap_pos =
        MagnetometerModel::heading_measurement(est_pos, MagnetometerModel::predict(true_neg, earth_ned), 0.1F);
    EXPECT_NEAR(wrap_pos.innovation, deg(20), 1e-4F);

    const auto wrap_neg =
        MagnetometerModel::heading_measurement(true_neg, MagnetometerModel::predict(est_pos, earth_ned), 0.1F);
    EXPECT_NEAR(wrap_neg.innovation, deg(-20), 1e-4F);

    // Field strength rejection.
    const float ref = earth_ned.norm();
    EXPECT_TRUE(MagnetometerModel::measure_heading(tilted, body_field, 0.1F, 0.0F, ref, 0.2F).has_value());
    EXPECT_FALSE(MagnetometerModel::measure_heading(tilted, body_field * 1.3F, 0.1F, 0.0F, ref, 0.2F).has_value());
}

}  // namespace
