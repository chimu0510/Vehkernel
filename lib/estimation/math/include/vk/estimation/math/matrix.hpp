// SPDX-License-Identifier: GPL-3.0-or-later
// Small fixed-size matrices for estimation (ADR-0014): plain single-precision
// values on the stack, with only the operations the EKF and its measurement
// models use.
#pragma once

#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace vk::estimation {

template <std::size_t Rows, std::size_t Cols>
struct Matrix {
    std::array<std::array<float, Cols>, Rows> data{};

    [[nodiscard]] static constexpr std::size_t rows() { return Rows; }
    [[nodiscard]] static constexpr std::size_t cols() { return Cols; }

    constexpr float& operator()(std::size_t r, std::size_t c) { return data[r][c]; }
    [[nodiscard]] constexpr float operator()(std::size_t r, std::size_t c) const { return data[r][c]; }

    [[nodiscard]] static constexpr Matrix zero() { return {}; }

    [[nodiscard]] static constexpr Matrix from_rows(const std::array<std::array<float, Cols>, Rows>& rows)
    {
        return {.data = rows};
    }

    [[nodiscard]] static constexpr Matrix identity()
        requires(Rows == Cols)
    {
        return diagonal(1.0F);
    }

    [[nodiscard]] static constexpr Matrix diagonal(float value)
        requires(Rows == Cols)
    {
        Matrix m{};
        for (std::size_t i = 0; i < Rows; ++i) {
            m(i, i) = value;
        }
        return m;
    }

    [[nodiscard]] static constexpr Matrix diagonal(const std::array<float, Rows>& values)
        requires(Rows == Cols)
    {
        Matrix m{};
        for (std::size_t i = 0; i < Rows; ++i) {
            m(i, i) = values[i];
        }
        return m;
    }

    [[nodiscard]] static constexpr Matrix from_vector(const Vector3& v)
        requires(Rows == 3 && Cols == 1)
    {
        return from_rows({{{{v.x}}, {{v.y}}, {{v.z}}}});
    }

    [[nodiscard]] constexpr Vector3 vector() const
        requires(Rows == 3 && Cols == 1)
    {
        return {data[0][0], data[1][0], data[2][0]};
    }

    // Skew-symmetric cross-product matrix [v x], so skew(u) * from_vector(v) = u x v.
    [[nodiscard]] static constexpr Matrix skew(const Vector3& v)
        requires(Rows == 3 && Cols == 3)
    {
        return from_rows({{
            {{0.0F, -v.z, v.y}},
            {{v.z, 0.0F, -v.x}},
            {{-v.y, v.x, 0.0F}},
        }});
    }

    // Rotation matrix from the body frame to the local level frame (NED) for a unit quaternion q.
    [[nodiscard]] static constexpr Matrix rotation(const Quaternion& q)
        requires(Rows == 3 && Cols == 3)
    {
        const float xx = q.x * q.x;
        const float yy = q.y * q.y;
        const float zz = q.z * q.z;
        const float xy = q.x * q.y;
        const float xz = q.x * q.z;
        const float yz = q.y * q.z;
        const float wx = q.w * q.x;
        const float wy = q.w * q.y;
        const float wz = q.w * q.z;
        return from_rows({{
            {{1.0F - 2.0F * (yy + zz), 2.0F * (xy - wz), 2.0F * (xz + wy)}},
            {{2.0F * (xy + wz), 1.0F - 2.0F * (xx + zz), 2.0F * (yz - wx)}},
            {{2.0F * (xz - wy), 2.0F * (yz + wx), 1.0F - 2.0F * (xx + yy)}},
        }});
    }

    template <std::size_t R, std::size_t C>
    constexpr void set_block(std::size_t row, std::size_t col, const Matrix<R, C>& sub)
        requires(R <= Rows && C <= Cols)
    {
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                data[row + r][col + c] = sub(r, c);
            }
        }
    }

    template <std::size_t R, std::size_t C>
    [[nodiscard]] constexpr Matrix<R, C> block(std::size_t row, std::size_t col) const
        requires(R <= Rows && C <= Cols)
    {
        Matrix<R, C> sub{};
        for (std::size_t r = 0; r < R; ++r) {
            for (std::size_t c = 0; c < C; ++c) {
                sub(r, c) = data[row + r][col + c];
            }
        }
        return sub;
    }

    [[nodiscard]] constexpr Matrix<Cols, Rows> transposed() const
    {
        Matrix<Cols, Rows> out{};
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t c = 0; c < Cols; ++c) {
                out(c, r) = data[r][c];
            }
        }
        return out;
    }

    // (M + M^T) / 2, with off-diagonal pairs assigned identically.
    [[nodiscard]] constexpr Matrix symmetric() const
        requires(Rows == Cols)
    {
        Matrix out{};
        for (std::size_t r = 0; r < Rows; ++r) {
            out(r, r) = data[r][r];
            for (std::size_t c = r + 1; c < Cols; ++c) {
                const float mean = 0.5F * (data[r][c] + data[c][r]);
                out(r, c) = mean;
                out(c, r) = mean;
            }
        }
        return out;
    }

    // Inverse of a 1x1, 2x2 or 3x3 matrix; nullopt when singular or non-finite.
    [[nodiscard]] std::optional<Matrix> inverted() const
        requires(Rows == Cols && (Rows == 1 || Rows == 2 || Rows == 3))
    {
        float scale = 0.0F;
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t c = 0; c < Cols; ++c) {
                if (!std::isfinite(data[r][c])) {
                    return std::nullopt;
                }
                scale = std::max(scale, std::abs(data[r][c]));
            }
        }
        if (!(scale > 0.0F)) {
            return std::nullopt;
        }
        if constexpr (Rows == 1) {
            return from_rows({{{{1.0F / data[0][0]}}}});
        } else if constexpr (Rows == 2) {
            const float det = data[0][0] * data[1][1] - data[0][1] * data[1][0];
            if (!std::isfinite(det) || std::abs(det) <= 1e-6F * scale * scale) {
                return std::nullopt;
            }
            const float inv = 1.0F / det;
            return from_rows({{
                {{data[1][1] * inv, -data[0][1] * inv}},
                {{-data[1][0] * inv, data[0][0] * inv}},
            }});
        } else {
            const float c00 = data[1][1] * data[2][2] - data[1][2] * data[2][1];
            const float c01 = -(data[1][0] * data[2][2] - data[1][2] * data[2][0]);
            const float c02 = data[1][0] * data[2][1] - data[1][1] * data[2][0];
            const float c10 = -(data[0][1] * data[2][2] - data[0][2] * data[2][1]);
            const float c11 = data[0][0] * data[2][2] - data[0][2] * data[2][0];
            const float c12 = -(data[0][0] * data[2][1] - data[0][1] * data[2][0]);
            const float c20 = data[0][1] * data[1][2] - data[0][2] * data[1][1];
            const float c21 = -(data[0][0] * data[1][2] - data[0][2] * data[1][0]);
            const float c22 = data[0][0] * data[1][1] - data[0][1] * data[1][0];
            const float det = data[0][0] * c00 + data[0][1] * c01 + data[0][2] * c02;
            if (!std::isfinite(det) || std::abs(det) <= 1e-6F * scale * scale * scale) {
                return std::nullopt;
            }
            const float inv = 1.0F / det;
            return from_rows({{
                {{c00 * inv, c10 * inv, c20 * inv}},
                {{c01 * inv, c11 * inv, c21 * inv}},
                {{c02 * inv, c12 * inv, c22 * inv}},
            }});
        }
    }

    constexpr Matrix& operator+=(const Matrix& o)
    {
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t c = 0; c < Cols; ++c) {
                data[r][c] += o(r, c);
            }
        }
        return *this;
    }

    constexpr Matrix& operator-=(const Matrix& o)
    {
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t c = 0; c < Cols; ++c) {
                data[r][c] -= o(r, c);
            }
        }
        return *this;
    }

    constexpr Matrix& operator*=(float s)
    {
        for (std::size_t r = 0; r < Rows; ++r) {
            for (std::size_t c = 0; c < Cols; ++c) {
                data[r][c] *= s;
            }
        }
        return *this;
    }
};

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] constexpr Matrix<Rows, Cols> operator+(Matrix<Rows, Cols> a, const Matrix<Rows, Cols>& b)
{
    return a += b;
}

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] constexpr Matrix<Rows, Cols> operator-(Matrix<Rows, Cols> a, const Matrix<Rows, Cols>& b)
{
    return a -= b;
}

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] constexpr Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols>& a)
{
    Matrix<Rows, Cols> out{};
    for (std::size_t r = 0; r < Rows; ++r) {
        for (std::size_t c = 0; c < Cols; ++c) {
            out(r, c) = -a(r, c);
        }
    }
    return out;
}

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] constexpr Matrix<Rows, Cols> operator*(Matrix<Rows, Cols> a, float s)
{
    return a *= s;
}

template <std::size_t Rows, std::size_t Cols>
[[nodiscard]] constexpr Matrix<Rows, Cols> operator*(float s, Matrix<Rows, Cols> a)
{
    return a *= s;
}

template <std::size_t R, std::size_t K, std::size_t C>
[[nodiscard]] constexpr Matrix<R, C> operator*(const Matrix<R, K>& a, const Matrix<K, C>& b)
{
    Matrix<R, C> out{};
    for (std::size_t r = 0; r < R; ++r) {
        for (std::size_t k = 0; k < K; ++k) {
            const float aik = a(r, k);
            for (std::size_t c = 0; c < C; ++c) {
                out(r, c) += aik * b(k, c);
            }
        }
    }
    return out;
}

using Matrix3 = Matrix<3, 3>;
using Matrix6 = Matrix<6, 6>;

}  // namespace vk::estimation
