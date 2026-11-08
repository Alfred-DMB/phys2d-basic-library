#pragma once
#include <algorithm>
#include <cmath>
#include <ostream>

namespace phys2d {

// phys2d/vec2.hpp — Vec2 2D en pixeles, header-only, sin allocs.
struct Vec2 {
    float x = 0.f;
    float y = 0.f;

    constexpr Vec2() noexcept = default;
    constexpr Vec2(float x_, float y_) noexcept : x(x_), y(y_) {
    }

    [[nodiscard]] constexpr Vec2 operator+(Vec2 other) const noexcept {
        return {x + other.x, y + other.y};
    }
    [[nodiscard]] constexpr Vec2 operator-(Vec2 other) const noexcept {
        return {x - other.x, y - other.y};
    }
    [[nodiscard]] constexpr Vec2 operator*(float s) const noexcept {
        return {x * s, y * s};
    }
    [[nodiscard]] constexpr Vec2 operator/(float s) const noexcept {
        return {x / s, y / s};
    }
    [[nodiscard]] constexpr Vec2 operator-() const noexcept {
        return {-x, -y};
    }
    constexpr Vec2& operator+=(Vec2 other) noexcept {
        x += other.x;
        y += other.y;
        return *this;
    }
    constexpr Vec2& operator-=(Vec2 other) noexcept {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    constexpr Vec2& operator*=(float s) noexcept {
        x *= s;
        y *= s;
        return *this;
    }
    constexpr Vec2& operator/=(float s) noexcept {
        x /= s;
        y /= s;
        return *this;
    }
    constexpr bool operator==(Vec2 const& other) const noexcept = default;

    [[nodiscard]] constexpr float dot(Vec2 other) const noexcept {
        return x * other.x + y * other.y;
    }
    [[nodiscard]] constexpr float cross(Vec2 other) const noexcept {
        return x * other.y - y * other.x;
    }
    [[nodiscard]] constexpr float length2() const noexcept {
        return dot(*this);
    }
    [[nodiscard]] constexpr float lengthSq() const noexcept {
        return length2();
    }
    [[nodiscard]] float length() const noexcept {
        return std::sqrt(length2());
    }
    [[nodiscard]] Vec2 normalized() const noexcept {
        float len = length();
        return len > 1e-8f ? (*this) / len : Vec2{};
    }
    [[nodiscard]] Vec2 normalizedOr(Vec2 fallback) const noexcept {
        float len = length();
        return len > 1e-8f ? (*this) / len : fallback;
    }
    [[nodiscard]] constexpr Vec2 perpendicular() const noexcept {
        return {-y, x};
    }
    [[nodiscard]] Vec2 rotated(float angle) const noexcept {
        float c = std::cos(angle);
        float s = std::sin(angle);
        return {x * c - y * s, x * s + y * c};
    }
    [[nodiscard]] constexpr Vec2 reflect(Vec2 normal) const noexcept {
        return (*this) - normal * (2.f * dot(normal));
    }
    [[nodiscard]] constexpr Vec2 lerp(Vec2 other, float t) const noexcept {
        return {x + (other.x - x) * t, y + (other.y - y) * t};
    }
    [[nodiscard]] constexpr Vec2 clamp(Vec2 lo, Vec2 hi) const noexcept {
        return {std::clamp(x, lo.x, hi.x), std::clamp(y, lo.y, hi.y)};
    }
    [[nodiscard]] float angle() const noexcept {
        return std::atan2(y, x);
    }
};

// Multiplicacion escalar por la izquierda.
[[nodiscard]] constexpr Vec2 operator*(float s, Vec2 v) noexcept {
    return v * s;
}
// Minimo componente a componente.
[[nodiscard]] constexpr Vec2 min(Vec2 a, Vec2 b) noexcept {
    return {std::min(a.x, b.x), std::min(a.y, b.y)};
}
// Maximo componente a componente.
[[nodiscard]] constexpr Vec2 max(Vec2 a, Vec2 b) noexcept {
    return {std::max(a.x, b.x), std::max(a.y, b.y)};
}
// Interpolacion lineal entre a y b.
[[nodiscard]] constexpr Vec2 lerp(Vec2 a, Vec2 b, float t) noexcept {
    return a.lerp(b, t);
}
// Clamp componente a componente de v a [lo, hi].
[[nodiscard]] constexpr Vec2 clamp(Vec2 v, Vec2 lo, Vec2 hi) noexcept {
    return v.clamp(lo, hi);
}
// Valor absoluto componente a componente.
[[nodiscard]] constexpr Vec2 abs(Vec2 v) noexcept {
    return {std::abs(v.x), std::abs(v.y)};
}
// Distancia euclidea entre a y b.
[[nodiscard]] float distance(Vec2 a, Vec2 b) noexcept {
    return (a - b).length();
}
// Distancia al cuadrado entre a y b.
[[nodiscard]] float distance2(Vec2 a, Vec2 b) noexcept {
    return (a - b).length2();
}

inline std::ostream& operator<<(std::ostream& os, Vec2 v) {
    return os << '(' << v.x << ", " << v.y << ')';
}

} // namespace phys2d
