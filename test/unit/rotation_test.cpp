// Mounting rotations (ADR-0014): each is a proper rotation, and a few known cases.
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

constexpr auto last = static_cast<std::uint8_t>(Rotation::Pitch270);

TEST(Rotation, EveryRotationIsProper)
{
    for (std::uint8_t r = 0; r <= last; ++r) {
        const auto rotation = static_cast<Rotation>(r);
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

}  // namespace
