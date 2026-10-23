// SPDX-License-Identifier: GPL-3.0-or-later
// Rotations (ADR-0014): each is a proper rotation, numbered as ArduPilot's
// table, a few known cases, and the composition of two.
#include <vk/hal/rotation.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace {

using vk::hal::Rotation;
using vk::hal::rotate;
using Vector = std::array<float, 3>;

constexpr float tolerance = 1e-6F;

void expect_vector(const Vector& actual, const Vector& expected)
{
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(actual.at(i), expected.at(i), tolerance) << "component " << i;
    }
}

float dot(const Vector& a, const Vector& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector cross(const Vector& a, const Vector& b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

TEST(Rotation, EveryRotationIsProper)
{
    for (const Rotation rotation : vk::hal::all_rotations) {
        const auto r = static_cast<std::uint8_t>(rotation);
        const Vector x = rotate(rotation, {1, 0, 0});
        const Vector y = rotate(rotation, {0, 1, 0});
        const Vector z = rotate(rotation, {0, 0, 1});
        SCOPED_TRACE(static_cast<int>(r));
        EXPECT_NEAR(dot(x, x), 1.0F, tolerance);
        EXPECT_NEAR(dot(x, y), 0.0F, tolerance);
        expect_vector(cross(x, y), z);  // right-handed, determinant +1
    }
}

TEST(Rotation, Roll180YawIsRoll180ThenYaw)
{
    const std::array<std::array<Rotation, 2>, 7> pairs{{
        {Rotation::Roll180Yaw45, Rotation::Yaw45},
        {Rotation::Roll180Yaw90, Rotation::Yaw90},
        {Rotation::Roll180Yaw135, Rotation::Yaw135},
        {Rotation::Pitch180, Rotation::Yaw180},
        {Rotation::Roll180Yaw225, Rotation::Yaw225},
        {Rotation::Roll180Yaw270, Rotation::Yaw270},
        {Rotation::Roll180Yaw315, Rotation::Yaw315},
    }};
    const Vector v{0.3F, -0.5F, 0.8F};
    for (const auto& [combined, yaw] : pairs) {
        expect_vector(rotate(combined, v), rotate(yaw, rotate(Rotation::Roll180, v)));
    }
}

TEST(Rotation, KnownCases)
{
    expect_vector(rotate(Rotation::None, {1, 2, 3}), {1, 2, 3});
    expect_vector(rotate(Rotation::Yaw90, {1, 0, 0}), {0, 1, 0});
    expect_vector(rotate(Rotation::Yaw180, {1, 2, 3}), {-1, -2, 3});
    expect_vector(rotate(Rotation::Roll180, {1, 2, 3}), {1, -2, -3});
    expect_vector(rotate(Rotation::Pitch90, {0, 0, 1}), {1, 0, 0});
}

TEST(Rotation, NumbersMatchArduPilotsTable)
{
    // ROTATION_* in ArduPilot's libraries/AP_Math/rotations.h, as PX4's SENS_BOARD_ROT.
    EXPECT_EQ(static_cast<int>(Rotation::None), 0);
    EXPECT_EQ(static_cast<int>(Rotation::Yaw90), 2);
    EXPECT_EQ(static_cast<int>(Rotation::Yaw315), 7);
    EXPECT_EQ(static_cast<int>(Rotation::Roll180), 8);
    EXPECT_EQ(static_cast<int>(Rotation::Pitch180), 12);
    EXPECT_EQ(static_cast<int>(Rotation::Roll180Yaw315), 15);
    EXPECT_EQ(static_cast<int>(Rotation::Roll90), 16);
    EXPECT_EQ(static_cast<int>(Rotation::Roll270), 20);
    EXPECT_EQ(static_cast<int>(Rotation::Pitch90), 24);
    EXPECT_EQ(static_cast<int>(Rotation::Pitch270), 25);
}

TEST(Rotation, ComposeAppliesTheFirstThenTheSecond)
{
    const Vector v{0.3F, -0.5F, 0.8F};
    for (const Rotation first : vk::hal::all_rotations) {
        for (const Rotation second : vk::hal::all_rotations) {
            SCOPED_TRACE(static_cast<int>(first) * 100 + static_cast<int>(second));
            expect_vector(vk::hal::compose(first, second) * v, rotate(second, rotate(first, v)));
        }
    }
}

TEST(Rotation, ComposingWithNoneChangesNothing)
{
    const Vector v{0.3F, -0.5F, 0.8F};
    for (const Rotation rotation : vk::hal::all_rotations) {
        expect_vector(vk::hal::compose(rotation, Rotation::None) * v, rotate(rotation, v));
        expect_vector(vk::hal::compose(Rotation::None, rotation) * v, rotate(rotation, v));
    }
    // A yaw of 90 deg twice is a yaw of 180 deg; roll 180 twice is no rotation.
    expect_vector(vk::hal::compose(Rotation::Yaw90, Rotation::Yaw90) * v, rotate(Rotation::Yaw180, v));
    expect_vector(vk::hal::compose(Rotation::Roll180, Rotation::Roll180) * v, v);
}

}  // namespace
