#pragma once
#include "phys2d/body.hpp"
#include "phys2d/shapes.hpp"
#include "phys2d/vec2.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <span>
#include <utility>
#include <variant>
#include <vector>

namespace phys2d {

// ─────────────────────────────────────────────────────────────────────────────
// query.hpp — queries sin depender de World (header-only, C++23, -Werror).
//
// Compatibilidad con la API de World:
//   - `World::bodiesSpan()` devuelve `std::span<Body*>` (no propietario).
//     Se acepta vía BodyPtrSpan (ver overloads de raycastPoint/pointQuery).
//   - `std::span<const std::unique_ptr<Body>>` sigue aceptándose por
//     conveniencia para vectores propios del llamante.
// ─────────────────────────────────────────────────────────────────────────────

struct RayHit {
    bool hit = false;
    Vec2 point{};
    Vec2 normal{};
    int bodyId = -1;
    float fraction = 1.f;
};

using RayCastResult = RayHit;
using BodySpan = std::span<std::unique_ptr<Body> const>;
using BodyPtrSpan = std::span<Body const*>;

// ── AABB / overlap ───────────────────────────────────────────────────────────

[[nodiscard]] inline bool overlapAABB(Vec2 aMin, Vec2 aMax, Vec2 bMin, Vec2 bMax) noexcept {
    return aMin.x <= bMax.x && bMin.x <= aMax.x && aMin.y <= bMax.y && bMin.y <= aMax.y;
}

[[nodiscard]] inline bool overlapAABB(std::pair<Vec2, Vec2> a, std::pair<Vec2, Vec2> b) noexcept {
    return overlapAABB(a.first, a.second, b.first, b.second);
}

[[nodiscard]] inline bool overlapAABB(AABB a, AABB b) noexcept {
    return overlapAABB(a.lower, a.upper, b.lower, b.upper);
}

[[nodiscard]] inline std::pair<Vec2, Vec2> bodyAABB(Body const& body) {
    AABB const a = shapeAABB(body.shape, body.pos, body.angle);
    return {a.lower, a.upper};
}

// ── Helpers primitivos ───────────────────────────────────────────────────────

[[nodiscard]] inline Vec2 closestPointOnSegment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2 ab = b - a;
    float const len2 = ab.length2();
    if (len2 <= 1e-12f)
        return a;
    float t = (p - a).dot(ab) / len2;
    t = std::clamp(t, 0.f, 1.f);
    return a + ab * t;
}

[[nodiscard]] inline Vec2 closestPointOnCircle(Vec2 p, Vec2 center, float radius) {
    Vec2 d = p - center;
    float const len2 = d.length2();
    if (len2 <= radius * radius)
        return p;
    float const len = std::sqrt(len2);
    return center + (d / len) * radius;
}

[[nodiscard]] inline Vec2 closestPointOnAABB(Vec2 p, Vec2 aMin, Vec2 aMax) {
    return {std::clamp(p.x, aMin.x, aMax.x), std::clamp(p.y, aMin.y, aMax.y)};
}

[[nodiscard]] inline Vec2 closestPointOnPolygon(Vec2 p, std::span<Vec2 const> vertsWorld) {
    if (vertsWorld.empty())
        return p;
    if (vertsWorld.size() == 1U)
        return vertsWorld[0];
    Vec2 best = closestPointOnSegment(p, vertsWorld[0], vertsWorld[1]);
    float bestDist2 = (p - best).length2();
    std::size_t const n = vertsWorld.size();
    for (std::size_t i = 1U; i < n; ++i) {
        Vec2 const a = vertsWorld[i];
        Vec2 const b = vertsWorld[(i + 1U) % n];
        Vec2 cand = closestPointOnSegment(p, a, b);
        float const d2 = (p - cand).length2();
        if (d2 < bestDist2) {
            bestDist2 = d2;
            best = cand;
        }
    }
    return best;
}

[[nodiscard]] inline Vec2 closestPointOnCapsule(Vec2 p, Vec2 a, Vec2 b, float radius) {
    Vec2 const core = closestPointOnSegment(p, a, b);
    Vec2 d = p - core;
    float const len2 = d.length2();
    if (len2 <= radius * radius)
        return p;
    float const len = std::sqrt(len2);
    return core + (d / len) * radius;
}

[[nodiscard]] inline bool pointInCircle(Vec2 p, Vec2 center, float radius) noexcept {
    return (p - center).length2() <= radius * radius;
}

[[nodiscard]] inline bool pointInBox(Vec2 p, Vec2 center, Vec2 half, float angle) {
    Vec2 const local = (p - center).rotated(-angle);
    return std::abs(local.x) <= half.x && std::abs(local.y) <= half.y;
}

[[nodiscard]] inline bool pointInPolygon(Vec2 p, std::span<Vec2 const> vertsWorld) noexcept {
    bool inside = false;
    std::size_t const n = vertsWorld.size();
    if (n < 3U)
        return false;
    for (std::size_t i = 0U, j = n - 1U; i < n; j = i++) {
        Vec2 const vi = vertsWorld[i];
        Vec2 const vj = vertsWorld[j];
        bool const crossed = ((vi.y > p.y) != (vj.y > p.y)) &&
                             (p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y) + vi.x);
        if (crossed)
            inside = !inside;
    }
    return inside;
}

[[nodiscard]] inline bool pointInCapsule(Vec2 p, Vec2 a, Vec2 b, float radius) {
    return (p - closestPointOnSegment(p, a, b)).length2() <= radius * radius;
}

// ── API pedida sobre Body (delega en shapes.hpp para cubrir las 5 formas) ───

[[nodiscard]] inline bool pointInShape(Vec2 point, Body const& body) {
    return pointInside(body.shape, point, body.pos, body.angle);
}

[[nodiscard]] inline Vec2 closestPointOnShape(Vec2 point, Body const& body) {
    Vec2 const local = worldToLocal(point, body.pos, body.angle);
    Vec2 const cLocal = closestPoint(body.shape, local);
    return localToWorld(cLocal, body.pos, body.angle);
}

// ── Ray primitivos (dir no necesita venir normalizado) ───────────────────────

[[nodiscard]] inline RayHit raycastCircle(Vec2 origin, Vec2 dir, float maxDist, Vec2 center,
                                          float radius) {
    RayHit out{};
    if (maxDist <= 0.f || radius < 0.f)
        return out;
    float const len = dir.length();
    if (len <= 1e-8f)
        return out;
    Vec2 const d = dir / len;
    Vec2 const oc = origin - center;
    float const c = oc.length2() - radius * radius;
    if (c < 0.f) {
        out.hit = true;
        out.point = origin;
        out.normal = -d;
        out.fraction = 0.f;
        return out;
    }
    float const b = oc.dot(d);
    float const disc = b * b - c;
    if (disc < 0.f)
        return out;
    float const t = -b - std::sqrt(disc);
    if (t < 0.f || t > maxDist)
        return out;
    Vec2 const p = origin + d * t;
    out.hit = true;
    out.point = p;
    out.normal = (p - center).normalized();
    out.fraction = t / maxDist;
    return out;
}

[[nodiscard]] inline RayHit raycastAABB(Vec2 origin, Vec2 dir, float maxDist, Vec2 aMin,
                                        Vec2 aMax) {
    RayHit out{};
    if (maxDist <= 0.f)
        return out;
    float const len = dir.length();
    if (len <= 1e-8f)
        return out;
    Vec2 const d = dir / len;
    if (origin.x >= aMin.x && origin.x <= aMax.x && origin.y >= aMin.y && origin.y <= aMax.y) {
        out.hit = true;
        out.point = origin;
        out.normal = -d;
        out.fraction = 0.f;
        return out;
    }
    float tMin = 0.f;
    float tMax = maxDist;
    Vec2 n{};
    auto const slab = [&tMin, &tMax, &n](float o, float dv, float mn, float mx, Vec2 axis) -> bool {
        if (std::abs(dv) <= 1e-8f)
            return o >= mn && o <= mx;
        float t1 = (mn - o) / dv;
        float t2 = (mx - o) / dv;
        Vec2 sign = dv > 0.f ? -axis : axis;
        if (t1 > t2) {
            float tmp = t1;
            t1 = t2;
            t2 = tmp;
        }
        if (t1 > tMin) {
            tMin = t1;
            n = sign;
        }
        tMax = std::min(tMax, t2);
        return tMin <= tMax;
    };
    if (!slab(origin.x, d.x, aMin.x, aMax.x, Vec2{1.f, 0.f}))
        return out;
    if (!slab(origin.y, d.y, aMin.y, aMax.y, Vec2{0.f, 1.f}))
        return out;
    out.hit = true;
    out.point = origin + d * tMin;
    out.normal = n;
    out.fraction = tMin / maxDist;
    return out;
}

[[nodiscard]] inline RayHit raycastPolygon(Vec2 origin, Vec2 dir, float maxDist,
                                           std::span<Vec2 const> vertsWorld) {
    RayHit out{};
    if (maxDist <= 0.f)
        return out;
    float const len = dir.length();
    if (len <= 1e-8f)
        return out;
    Vec2 const d = dir / len;
    std::size_t const n = vertsWorld.size();
    if (n < 3U)
        return out;
    if (pointInPolygon(origin, vertsWorld)) {
        out.hit = true;
        out.point = origin;
        out.normal = -d;
        out.fraction = 0.f;
        return out;
    }
    float bestT = maxDist;
    Vec2 bestN{};
    Vec2 bestP{};
    bool found = false;
    for (std::size_t i = 0U; i < n; ++i) {
        Vec2 const vi = vertsWorld[i];
        Vec2 const vj = vertsWorld[(i + 1U) % n];
        Vec2 const e = vj - vi;
        float const denom = d.cross(e);
        if (std::abs(denom) <= 1e-12f)
            continue;
        Vec2 const w = vi - origin;
        float const t = w.cross(e) / denom;
        float const u = w.cross(d) / denom;
        if (t < 0.f || t > maxDist || u < 0.f || u > 1.f)
            continue;
        if (t <= bestT) {
            bestT = t;
            Vec2 nn = e.perpendicular();
            if (nn.dot(d) > 0.f)
                nn = -nn;
            bestN = nn.normalized();
            bestP = origin + d * t;
            found = true;
        }
    }
    if (!found)
        return out;
    out.hit = true;
    out.point = bestP;
    out.normal = bestN;
    out.fraction = bestT / maxDist;
    return out;
}

[[nodiscard]] inline RayHit raycastSegment(Vec2 origin, Vec2 dir, float maxDist, Vec2 a, Vec2 b) {
    RayHit out{};
    if (maxDist <= 0.f)
        return out;
    float const len = dir.length();
    if (len <= 1e-8f)
        return out;
    Vec2 const d = dir / len;
    Vec2 const e = b - a;
    float const denom = d.cross(e);
    if (std::abs(denom) <= 1e-12f)
        return out;
    Vec2 const w = a - origin;
    float const t = w.cross(e) / denom;
    float const u = w.cross(d) / denom;
    if (t < 0.f || t > maxDist || u < 0.f || u > 1.f)
        return out;
    Vec2 nn = e.perpendicular();
    if (nn.dot(d) > 0.f)
        nn = -nn;
    out.hit = true;
    out.point = origin + d * t;
    out.normal = nn.normalized();
    out.fraction = t / maxDist;
    return out;
}

[[nodiscard]] inline RayHit raycastCapsule(Vec2 origin, Vec2 dir, float maxDist, Vec2 a, Vec2 b,
                                           float radius) {
    RayHit out{};
    if (maxDist <= 0.f)
        return out;
    float const len = dir.length();
    if (len <= 1e-8f)
        return out;
    Vec2 const d = dir / len;
    Vec2 const ab = b - a;
    float const segLen = ab.length();
    if (segLen <= 1e-8f)
        return raycastCircle(origin, dir, maxDist, a, radius);
    Vec2 const axis = ab / segLen;
    Vec2 const perp = axis.perpendicular();
    if (pointInCapsule(origin, a, b, radius)) {
        out.hit = true;
        out.point = origin;
        out.normal = -d;
        out.fraction = 0.f;
        return out;
    }
    bool found = false;
    float bestT = maxDist;
    Vec2 bestP{};
    Vec2 bestN{};
    auto considerCap = [&](Vec2 center, bool isA) {
        RayHit const h = raycastCircle(origin, d, maxDist, center, radius);
        if (!h.hit)
            return;
        float const t = h.fraction * maxDist;
        float const local = (h.point - a).dot(axis);
        // La mitad interior del círculo no es borde: solo la mitad exterior.
        bool outward = (isA && local <= 1e-4f) || (!isA && local >= segLen - 1e-4f);
        if (!outward)
            return;
        if (t <= bestT) {
            bestT = t;
            bestP = h.point;
            bestN = h.normal;
            found = true;
        }
    };
    considerCap(a, true);
    considerCap(b, false);
    auto considerSide = [&](Vec2 s0, Vec2 s1) {
        Vec2 const e = s1 - s0;
        float const denom = d.cross(e);
        if (std::abs(denom) <= 1e-12f)
            return;
        Vec2 const w = s0 - origin;
        float const t = w.cross(e) / denom;
        float const u = w.cross(d) / denom;
        if (t < 0.f || t > maxDist || u < 0.f || u > 1.f)
            return;
        if (t <= bestT) {
            Vec2 nn = e.perpendicular();
            if (nn.dot(d) > 0.f)
                nn = -nn;
            bestT = t;
            bestP = origin + d * t;
            bestN = nn.normalized();
            found = true;
        }
    };
    considerSide(a + perp * radius, b + perp * radius);
    considerSide(a - perp * radius, b - perp * radius);
    if (!found)
        return out;
    out.hit = true;
    out.point = bestP;
    out.normal = bestN;
    out.fraction = bestT / maxDist;
    return out;
}

// ── Ray sobre Body y sobre el conjunto de Bodies (sin World) ───────────────

[[nodiscard]] inline RayHit raycastBody(Vec2 origin, Vec2 dir, float maxDist, Body const& body) {
    return std::visit(
        [&origin, &dir, maxDist, &body](auto const& s) -> RayHit {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return raycastCircle(origin, dir, maxDist, body.pos, s.radius);
            } else if constexpr (std::is_same_v<T, AABB>) {
                Vec2 const corners[4] = {s.lower, s.upper, Vec2{s.lower.x, s.upper.y},
                                         Vec2{s.upper.x, s.lower.y}};
                Vec2 world[4]{};
                for (int i = 0; i < 4; ++i)
                    world[i] = corners[i].rotated(body.angle) + body.pos;
                std::span<Vec2 const> verts{world, 4U};
                return raycastPolygon(origin, dir, maxDist, verts);
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                std::vector<Vec2> world{};
                world.reserve(s.verts.size());
                for (Vec2 v : s.verts)
                    world.push_back(v.rotated(body.angle) + body.pos);
                return raycastPolygon(origin, dir, maxDist, std::span<Vec2 const>{world});
            } else if constexpr (std::is_same_v<T, Capsule>) {
                Vec2 const wa = s.capA().rotated(body.angle) + body.pos;
                Vec2 const wb = s.capB().rotated(body.angle) + body.pos;
                return raycastCapsule(origin, dir, maxDist, wa, wb, s.radius);
            } else {
                Vec2 const wa = s.a.rotated(body.angle) + body.pos;
                Vec2 const wb = s.b.rotated(body.angle) + body.pos;
                if (s.radius <= 1e-8f)
                    return raycastSegment(origin, dir, maxDist, wa, wb);
                return raycastCapsule(origin, dir, maxDist, wa, wb, s.radius);
            }
        },
        body.shape);
}

[[nodiscard]] inline RayHit raycastPoint(Vec2 origin, Vec2 dir, float maxDist, BodySpan bodies) {
    RayHit best{};
    if (maxDist <= 0.f)
        return best;
    if (dir.length() <= 1e-8f)
        return best;
    float bestT = maxDist;
    std::size_t idx = 0U;
    for (auto const& bp : bodies) {
        if (bp) {
            RayHit h = raycastBody(origin, dir, maxDist, *bp);
            if (h.hit) {
                float const t = h.fraction * maxDist;
                if (t <= bestT) {
                    bestT = t;
                    best = h;
                    best.bodyId = static_cast<int>(idx);
                }
            }
        }
        ++idx;
    }
    return best;
}

[[nodiscard]] inline RayHit raycastPoint(Vec2 origin, Vec2 dir, float maxDist, BodyPtrSpan bodies) {
    RayHit best{};
    if (maxDist <= 0.f)
        return best;
    if (dir.length() <= 1e-8f)
        return best;
    float bestT = maxDist;
    std::size_t idx = 0U;
    for (Body const* bp : bodies) {
        if (bp != nullptr) {
            RayHit h = raycastBody(origin, dir, maxDist, *bp);
            if (h.hit) {
                float const t = h.fraction * maxDist;
                if (t <= bestT) {
                    bestT = t;
                    best = h;
                    best.bodyId = static_cast<int>(idx);
                }
            }
        }
        ++idx;
    }
    return best;
}

[[nodiscard]] inline RayHit raycastWorld(Vec2 origin, Vec2 dir, float maxDist, BodySpan bodies) {
    return raycastPoint(origin, dir, maxDist, bodies);
}

[[nodiscard]] inline int pointQuery(Vec2 point, BodySpan bodies) {
    std::size_t idx = 0U;
    for (auto const& bp : bodies) {
        if (bp && pointInShape(point, *bp))
            return static_cast<int>(idx);
        ++idx;
    }
    return -1;
}

[[nodiscard]] inline int pointQuery(Vec2 point, BodyPtrSpan bodies) {
    std::size_t idx = 0U;
    for (Body const* bp : bodies) {
        if (bp != nullptr && pointInShape(point, *bp))
            return static_cast<int>(idx);
        ++idx;
    }
    return -1;
}

// ── Shape cast: barrido de un círculo (dir no normalizado, maxDist) ──────────
// Equivale a raycastCircle con radio suma sobre el centro del círculo barrido.
[[nodiscard]] inline RayHit shapeCastCircle(Vec2 start, Vec2 dir, float maxDist, float radius,
                                            Vec2 targetCenter, float targetRadius) {
    return raycastCircle(start, dir, maxDist, targetCenter,
                         targetRadius + (radius > 0.f ? radius : 0.f));
}

[[nodiscard]] inline RayHit shapeCastBody(Vec2 start, Vec2 dir, float maxDist, float radius,
                                          Body const& target) {
    if (radius < 0.f)
        radius = 0.f;
    return std::visit(
        [&start, &dir, maxDist, radius, &target](auto const& s) -> RayHit {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return shapeCastCircle(start, dir, maxDist, radius, target.pos, s.radius);
            } else if constexpr (std::is_same_v<T, AABB>) {
                if (target.angle == 0.f) {
                    // AABB expandido por radius: conservador en las esquinas.
                    Vec2 const ext{radius, radius};
                    return raycastAABB(start, dir, maxDist, s.lower + target.pos - ext,
                                       s.upper + target.pos + ext);
                }
                Vec2 const corners[4] = {s.lower, s.upper, Vec2{s.lower.x, s.upper.y},
                                         Vec2{s.upper.x, s.lower.y}};
                Vec2 world[4]{};
                for (int i = 0; i < 4; ++i)
                    world[i] = corners[i].rotated(target.angle) + target.pos;
                // Aproximación: barrido del centro sin engordar (sin expandir por radius).
                return raycastPolygon(start, dir, maxDist, std::span<Vec2 const>{world, 4U});
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                std::vector<Vec2> world{};
                world.reserve(s.verts.size());
                for (Vec2 v : s.verts)
                    world.push_back(v.rotated(target.angle) + target.pos);
                // Aproximación: barrido del centro sin engordar (sin expandir por radius).
                return raycastPolygon(start, dir, maxDist, std::span<Vec2 const>{world});
            } else if constexpr (std::is_same_v<T, Capsule>) {
                Vec2 const wa = s.capA().rotated(target.angle) + target.pos;
                Vec2 const wb = s.capB().rotated(target.angle) + target.pos;
                return raycastCapsule(start, dir, maxDist, wa, wb, s.radius + radius);
            } else {
                Vec2 const wa = s.a.rotated(target.angle) + target.pos;
                Vec2 const wb = s.b.rotated(target.angle) + target.pos;
                // Un segmento siempre se barre como cápsula engordada por el
                // radio (con radio 0 es la línea).
                return raycastCapsule(start, dir, maxDist, wa, wb, s.radius + radius);
            }
        },
        target.shape);
}

// ── Shape cast exacto convexo-vs-convexo por Minkowski (solo traslacion) ────
// convexHull: monotone chain; devuelve el casco CCW. Si <3 puntos o area ~0,
// devuelve los puntos ordenados (sin duplicados exactos).
[[nodiscard]] inline std::vector<Vec2> convexHull(std::span<Vec2 const> points) {
    std::vector<Vec2> pts{points.begin(), points.end()};
    std::sort(pts.begin(), pts.end(), [](Vec2 const& a, Vec2 const& b) {
        if (a.x != b.x)
            return a.x < b.x;
        return a.y < b.y;
    });
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    if (pts.size() < 3U)
        return pts;
    auto const cross = [](Vec2 const& o, Vec2 const& a, Vec2 const& b) -> float {
        return (a - o).cross(b - a);
    };
    std::vector<Vec2> lower{};
    std::vector<Vec2> upper{};
    lower.reserve(pts.size());
    upper.reserve(pts.size());
    for (Vec2 const& p : pts) {
        while (lower.size() >= 2U && cross(lower[lower.size() - 2U], lower.back(), p) <= 0.f)
            lower.pop_back();
        lower.push_back(p);
    }
    for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
        while (upper.size() >= 2U && cross(upper[upper.size() - 2U], upper.back(), *it) <= 0.f)
            upper.pop_back();
        upper.push_back(*it);
    }
    lower.pop_back();
    upper.pop_back();
    std::vector<Vec2> hull = lower;
    hull.insert(hull.end(), upper.begin(), upper.end());
    if (hull.size() < 3U)
        return pts;
    float area2 = 0.f;
    for (std::size_t i = 0U; i < hull.size(); ++i)
        area2 += hull[i].cross(hull[(i + 1U) % hull.size()]);
    if (std::abs(area2) <= 1e-12f)
        return pts;
    return hull;
}

// shapeCastShape: `movingVerts` son RELATIVOS a `start` (el poligono movible en
// t=0 es start+movingVerts), `targetVerts` estan en mundo estaticas; el contacto
// es cuando start+dir*t entra en D'=casco(targetVerts-movingVerts).
[[nodiscard]] inline RayHit shapeCastShape(Vec2 start, Vec2 dir, float maxDist,
                                           std::span<Vec2 const> movingVerts,
                                           std::span<Vec2 const> targetVerts) {
    RayHit out{};
    if (maxDist <= 0.f || dir.length() <= 1e-8f)
        return out;
    if (movingVerts.empty() || targetVerts.empty())
        return out;
    std::vector<Vec2> diffs{};
    diffs.reserve(movingVerts.size() * targetVerts.size());
    for (Vec2 const& tVert : targetVerts) {
        for (Vec2 const& mVert : movingVerts)
            diffs.push_back(tVert - mVert);
    }
    std::vector<Vec2> mink = convexHull(std::span<Vec2 const>{diffs});
    if (mink.size() < 3U)
        return out;
    return raycastPolygon(start, dir, maxDist, std::span<Vec2 const>{mink});
}

// shapeCastBodyShape: convenience sobre dos Body; extrae verts mundo (target) y
// relativos a moving.pos (movible) si ambos son AABB o ConvexPolygon. Si alguna
// forma es otra (Circulo/Capsula/Segmento) devuelve miss (hit=false).
[[nodiscard]] inline RayHit shapeCastBodyShape(Vec2 dir, float maxDist, Body const& moving,
                                               Body const& target) {
    std::vector<Vec2> movingRel{};
    std::vector<Vec2> targetWorld{};
    bool movingOk = false;
    bool targetOk = false;
    std::visit(
        [&moving, &movingRel, &movingOk](auto const& s) {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, AABB>) {
                Vec2 const corners[4] = {s.lower, s.upper, Vec2{s.lower.x, s.upper.y},
                                         Vec2{s.upper.x, s.lower.y}};
                for (int i = 0; i < 4; ++i)
                    movingRel.push_back(corners[i].rotated(moving.angle));
                movingOk = true;
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                movingRel.reserve(s.verts.size());
                for (Vec2 v : s.verts)
                    movingRel.push_back(v.rotated(moving.angle));
                movingOk = !s.verts.empty();
            } else {
                movingOk = false;
            }
        },
        moving.shape);
    std::visit(
        [&target, &targetWorld, &targetOk](auto const& s) {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, AABB>) {
                Vec2 const corners[4] = {s.lower, s.upper, Vec2{s.lower.x, s.upper.y},
                                         Vec2{s.upper.x, s.lower.y}};
                for (int i = 0; i < 4; ++i)
                    targetWorld.push_back(corners[i].rotated(target.angle) + target.pos);
                targetOk = true;
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                targetWorld.reserve(s.verts.size());
                for (Vec2 v : s.verts)
                    targetWorld.push_back(v.rotated(target.angle) + target.pos);
                targetOk = !s.verts.empty();
            } else {
                targetOk = false;
            }
        },
        target.shape);
    if (!movingOk || !targetOk)
        return RayHit{};
    return shapeCastShape(moving.pos, dir, maxDist, std::span<Vec2 const>{movingRel},
                          std::span<Vec2 const>{targetWorld});
}

[[nodiscard]] inline RayHit shapeCastPoint(Vec2 start, Vec2 dir, float maxDist, float radius,
                                           BodySpan bodies) {
    RayHit best{};
    if (maxDist <= 0.f)
        return best;
    if (dir.length() <= 1e-8f)
        return best;
    float bestT = maxDist;
    std::size_t idx = 0U;
    for (auto const& bp : bodies) {
        if (bp) {
            RayHit h = shapeCastBody(start, dir, maxDist, radius, *bp);
            if (h.hit) {
                float const t = h.fraction * maxDist;
                if (t <= bestT) {
                    bestT = t;
                    best = h;
                    best.bodyId = static_cast<int>(idx);
                }
            }
        }
        ++idx;
    }
    return best;
}

} // namespace phys2d
