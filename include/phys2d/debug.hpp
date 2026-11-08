#pragma once
#include "phys2d/body.hpp"
#include "phys2d/shapes.hpp"
#include "phys2d/vec2.hpp"
#include <memory>
#include <span>
#include <variant>
#include <vector>

namespace phys2d {

// ─────────────────────────────────────────────────────────────────────────────
// debug.hpp — DebugDraw virtual + helper drawWorld (header-only, C++23).
// Sin depender de World: toma std::span<Body const*> (compatible con
// World::bodiesSpan()) u overloads desde std::span<const std::unique_ptr<Body>>
// o std::span<Body*>.
// ─────────────────────────────────────────────────────────────────────────────

struct DebugDraw {
    virtual ~DebugDraw() = default;
    virtual void drawCircle(Vec2 center, float radius) = 0;
    virtual void drawBox(Vec2 center, Vec2 half, float angle) = 0;
    virtual void drawSegment(Vec2 a, Vec2 b) = 0;
    virtual void drawPolygon(std::span<Vec2 const> vertsWorld) {
        if (vertsWorld.size() < 2U) {
            return;
        }
        for (std::size_t i = 0U; i < vertsWorld.size(); ++i) {
            drawSegment(vertsWorld[i], vertsWorld[(i + 1U) % vertsWorld.size()]);
        }
    }
    virtual void drawCapsule(Vec2 a, Vec2 b, float radius) {
        drawCircle(a, radius);
        drawCircle(b, radius);
        Vec2 const perp = (b - a).normalized().perpendicular();
        drawSegment(a + perp * radius, b + perp * radius);
        drawSegment(a - perp * radius, b - perp * radius);
    }
};

[[nodiscard]] inline bool drawBody(DebugDraw& dd, Body const& body) {
    return std::visit(
        [&dd, &body](auto const& s) -> bool {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                dd.drawCircle(body.pos, s.radius);
            } else if constexpr (std::is_same_v<T, AABB>) {
                // El centro de la AABB puede no coincidir con Body::pos
                // (p. ej. AABB::fromLowerUpper); hay que rotar el offset.
                Vec2 const center = body.pos + s.center().rotated(body.angle);
                dd.drawBox(center, s.half(), body.angle);
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                std::vector<Vec2> world;
                world.reserve(s.verts.size());
                for (Vec2 v : s.verts) {
                    world.push_back(v.rotated(body.angle) + body.pos);
                }
                dd.drawPolygon(std::span<Vec2 const>{world});
            } else if constexpr (std::is_same_v<T, Capsule>) {
                Vec2 const wa = s.capA().rotated(body.angle) + body.pos;
                Vec2 const wb = s.capB().rotated(body.angle) + body.pos;
                dd.drawCapsule(wa, wb, s.radius);
            } else {
                Vec2 const wa = s.a.rotated(body.angle) + body.pos;
                Vec2 const wb = s.b.rotated(body.angle) + body.pos;
                dd.drawSegment(wa, wb);
            }
            return true;
        },
        body.shape);
}

inline void drawWorld(DebugDraw& dd, std::span<std::unique_ptr<Body> const> bodies) {
    for (auto const& bp : bodies) {
        if (bp)
            (void)drawBody(dd, *bp);
    }
}

inline void drawWorld(DebugDraw& dd, std::span<Body const*> bodies) {
    for (Body const* bp : bodies) {
        if (bp != nullptr)
            (void)drawBody(dd, *bp);
    }
}

} // namespace phys2d
