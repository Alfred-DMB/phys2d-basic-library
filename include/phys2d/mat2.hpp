#pragma once
#include "phys2d/vec2.hpp"

namespace phys2d {

// phys2d/mat2.hpp — Mat2 columna-major: M * v = col0 * v.x + col1 * v.y.
struct Mat2 {
    Vec2 col0{1.f, 0.f};
    Vec2 col1{0.f, 1.f};

    constexpr Mat2() noexcept = default;
    constexpr Mat2(Vec2 c0_, Vec2 c1_) noexcept : col0(c0_), col1(c1_) {
    }
    constexpr Mat2(float m00, float m01, float m10, float m11) noexcept
        : col0{m00, m10}, col1{m01, m11} {
    }

    [[nodiscard]] constexpr bool operator==(Mat2 const& other) const noexcept = default;
    [[nodiscard]] constexpr Vec2 row0() const noexcept {
        return {col0.x, col1.x};
    }
    [[nodiscard]] constexpr Vec2 row1() const noexcept {
        return {col0.y, col1.y};
    }
    [[nodiscard]] constexpr Vec2 operator*(Vec2 v) const noexcept {
        return col0 * v.x + col1 * v.y;
    }
    [[nodiscard]] constexpr Mat2 operator*(Mat2 other) const noexcept {
        return {(*this) * other.col0, (*this) * other.col1};
    }
    [[nodiscard]] constexpr Mat2 operator*(float s) const noexcept {
        return {col0 * s, col1 * s};
    }
    constexpr Mat2& operator*=(float s) noexcept {
        col0 *= s;
        col1 *= s;
        return *this;
    }
    constexpr Mat2& operator*=(Mat2 other) noexcept {
        *this = (*this) * other;
        return *this;
    }

    [[nodiscard]] static constexpr Mat2 identity() noexcept {
        return {};
    }
    [[nodiscard]] static constexpr Mat2 zero() noexcept {
        return {Vec2{}, Vec2{}};
    }
    [[nodiscard]] static Mat2 rotation(float angle) noexcept {
        float c = std::cos(angle);
        float s = std::sin(angle);
        return {{c, s}, {-s, c}};
    }
    [[nodiscard]] static Mat2 fromAngle(float angle) noexcept {
        return rotation(angle);
    }
    [[nodiscard]] static constexpr Mat2 scaling(Vec2 scale) noexcept {
        return {{scale.x, 0.f}, {0.f, scale.y}};
    }
    [[nodiscard]] static constexpr Mat2 fromColumns(Vec2 c0_, Vec2 c1_) noexcept {
        return {c0_, c1_};
    }
    [[nodiscard]] static constexpr Mat2 fromRows(Vec2 r0_, Vec2 r1_) noexcept {
        return {{r0_.x, r1_.x}, {r0_.y, r1_.y}};
    }
    [[nodiscard]] constexpr float determinant() const noexcept {
        return col0.x * col1.y - col0.y * col1.x;
    }
    [[nodiscard]] constexpr Mat2 transpose() const noexcept {
        return {{col0.x, col1.x}, {col0.y, col1.y}};
    }
    [[nodiscard]] constexpr Mat2 inverse() const noexcept {
        float d = determinant();
        float inv = d != 0.f ? 1.f / d : 0.f;
        return {{col1.y * inv, -col0.y * inv}, {-col1.x * inv, col0.x * inv}};
    }
};

// Multiplicacion escalar por la izquierda.
[[nodiscard]] constexpr Mat2 operator*(float s, Mat2 m) noexcept {
    return m * s;
}

} // namespace phys2d
