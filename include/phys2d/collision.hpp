#pragma once
#include "phys2d/body.hpp"
#include "phys2d/shapes.hpp"
#include "phys2d/vec2.hpp"
#if __has_include("phys2d/mat2.hpp")
#include "phys2d/mat2.hpp"
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <span>
#include <utility>
#include <variant>
#include <vector>

// Narrowphase for phys2d. Convention (fundamental, do not invert):
//   Manifold::normal always points from body A to body B (A -> B).
// Every pairwise function below is order-agnostic on input (it detects
// which argument holds which shape) and order-aware on output: the
// returned normal is A -> B in BOTH argument orders. The dispatcher
// collide(A, B, out) therefore must NOT flip the normal after calling
// a pairwise function. Example: floor (box, A) -> ball (circle, B)
// with the ball above the floor yields normal (0, -1) in this
// y-down world (gravity +y), and the solver pushes B (the ball)
// along +normal, i.e. upwards, out of the floor. The reversed call
// collide(ball, floor, out) yields (0, +1) (ball -> floor).
//
// Supported pairs (dispatcher):
//   Circle-Circle, Circle-Box/AABB, Circle-Polygon, Circle-Capsule,
//   Circle-Segment, Box-Box, Box/AABB-Polygon, Polygon-Polygon,
//   Capsule-Capsule, Capsule-Polygon, Capsule-Box/AABB,
//   Capsule-Segment, Segment-Segment, Segment-Polygon/Box/AABB.
// Polygon-Polygon / Box-Box use full SAT over both polygons' face
// axes, minimum-overlap axis, and a reference/incident face clip
// that yields up to 2 contact points. Circle/capsule/segment pairs
// use closest-feature tests and honestly report count == 1.
// Segment pairs have no area: depth is 0 and they only collide on
// intersection (segment-segment) or touching (capsule radius 0).
//
// The Shape variant has exactly 5 types (Circle, AABB/Box,
// ConvexPolygon, Capsule, Segment, see shapes.hpp). detail::kindOf
// classifies them directly with std::holds_alternative and the
// detail::extract*Geom helpers use std::get_if on the concrete
// type; there is no generic duck-typing. Local shape data is in
// body local space and transformed by Body::pos / Body::angle.

namespace phys2d {

struct Manifold {
    Vec2 normal{1.f, 0.f};
    float depth = 0.f;
    Vec2 point{};
    Vec2 point2{};
    int count = 0;
    float restitution = 0.f;
    float friction = 0.f;
    // Compatibility with the previous Contact used by world.hpp.
    int a = -1;
    int b = -1;
    Vec2 relVel{};
};

using Contact = Manifold;

struct Projection {
    float min = 0.f;
    float max = 0.f;
};

// ------------------------------------------------------------------
// SAT helpers
// ------------------------------------------------------------------

[[nodiscard]] inline Projection project(std::span<const Vec2> verts, Vec2 axis) noexcept {
    if (verts.empty()) {
        return Projection{0.f, 0.f};
    }
    float lo = verts[0].dot(axis);
    float hi = lo;
    for (std::size_t i = 1; i < verts.size(); ++i) {
        float d = verts[i].dot(axis);
        if (d < lo) {
            lo = d;
        }
        if (d > hi) {
            hi = d;
        }
    }
    return Projection{lo, hi};
}

[[nodiscard]] inline Vec2 centroidOf(std::span<const Vec2> verts) noexcept {
    if (verts.empty()) {
        return Vec2{};
    }
    Vec2 sum{};
    for (auto const& v : verts) {
        sum += v;
    }
    return sum / static_cast<float>(verts.size());
}

// Face normal of the convex polygon (verts = px) most aligned with
// `toward`. Oriented outwards (away from the polygon centroid).
[[nodiscard]] inline Vec2 findAxis(std::span<const Vec2> verts, Vec2 toward) noexcept {
    Vec2 fallback = toward.normalized();
    if (verts.size() < 2) {
        return fallback.x == 0.f && fallback.y == 0.f ? Vec2{1.f, 0.f} : fallback;
    }
    Vec2 centroid = centroidOf(verts);
    Vec2 best = fallback;
    float bestDot = -std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < verts.size(); ++i) {
        Vec2 v1 = verts[i];
        Vec2 v2 = verts[(i + 1) % verts.size()];
        Vec2 edge = v2 - v1;
        Vec2 n = edge.perpendicular();
        float len = n.length();
        if (len < 1e-8f) {
            continue;
        }
        n = n / len;
        if (n.dot(centroid - v1) > 0.f) {
            n = -n;
        }
        float d = n.dot(toward);
        if (d > bestDot) {
            bestDot = d;
            best = n;
        }
    }
    return best;
}

// Incident face: the 2 vertices of the edge whose outward normal is
// most anti-parallel to referenceNormal.
[[nodiscard]] inline std::array<Vec2, 2> findIncidentFace(std::span<const Vec2> verts,
                                                          Vec2 referenceNormal) noexcept {
    if (verts.empty()) {
        return {Vec2{}, Vec2{}};
    }
    if (verts.size() == 1) {
        return {verts[0], verts[0]};
    }
    Vec2 centroid = centroidOf(verts);
    std::size_t bestIdx = 0;
    float bestDot = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < verts.size(); ++i) {
        Vec2 v1 = verts[i];
        Vec2 v2 = verts[(i + 1) % verts.size()];
        Vec2 n = (v2 - v1).perpendicular();
        float len = n.length();
        if (len < 1e-8f) {
            continue;
        }
        n = n / len;
        if (n.dot(centroid - v1) > 0.f) {
            n = -n;
        }
        float d = n.dot(referenceNormal);
        if (d < bestDot) {
            bestDot = d;
            bestIdx = i;
        }
    }
    return {verts[bestIdx], verts[(bestIdx + 1) % verts.size()]};
}

inline void findIncidentFace(std::span<const Vec2> verts, Vec2 referenceNormal, Vec2& outA,
                             Vec2& outB) noexcept {
    auto face = findIncidentFace(verts, referenceNormal);
    outA = face[0];
    outB = face[1];
}

// Clip a segment against a plane, keeping the side where
// dot(p - planePoint, planeNormal) <= 0. Returns the clipped
// segment; if fully outside, a degenerate segment at the first
// point is returned (use the span overload to test emptiness).
[[nodiscard]] inline std::array<Vec2, 2> clipSegment(Vec2 p1, Vec2 p2, Vec2 planePoint,
                                                     Vec2 planeNormal) noexcept {
    float d1 = (p1 - planePoint).dot(planeNormal);
    float d2 = (p2 - planePoint).dot(planeNormal);
    bool in1 = d1 <= 0.f;
    bool in2 = d2 <= 0.f;
    if (in1 && in2) {
        return {p1, p2};
    }
    if (!in1 && !in2) {
        return {p1, p1};
    }
    float denom = d1 - d2;
    float t = std::abs(denom) < 1e-12f ? 0.f : d1 / denom;
    Vec2 hit = p1 + (p2 - p1) * t;
    if (in1) {
        return {p1, hit};
    }
    return {hit, p2};
}

// Span overload: Sutherland-style plane clip. A 2-point input is a
// segment, a longer input is treated as a polygon loop. Returns
// 0, 1 or 2+ points; empty means fully clipped away.
[[nodiscard]] inline std::vector<Vec2> clipSegment(std::span<const Vec2> in, Vec2 planePoint,
                                                   Vec2 planeNormal) {
    std::vector<Vec2> out;
    if (in.empty()) {
        return out;
    }
    if (in.size() == 2) {
        float d1 = (in[0] - planePoint).dot(planeNormal);
        float d2 = (in[1] - planePoint).dot(planeNormal);
        bool in1 = d1 <= 0.f;
        bool in2 = d2 <= 0.f;
        if (in1 && in2) {
            out.push_back(in[0]);
            out.push_back(in[1]);
        } else if (in1 || in2) {
            auto clipped = clipSegment(in[0], in[1], planePoint, planeNormal);
            out.push_back(clipped[0]);
            out.push_back(clipped[1]);
        }
        return out;
    }
    for (std::size_t i = 0; i < in.size(); ++i) {
        Vec2 cur = in[i];
        Vec2 prev = in[(i + in.size() - 1) % in.size()];
        float dCur = (cur - planePoint).dot(planeNormal);
        float dPrev = (prev - planePoint).dot(planeNormal);
        bool curIn = dCur <= 0.f;
        bool prevIn = dPrev <= 0.f;
        if (curIn && prevIn) {
            out.push_back(cur);
        } else if (prevIn && !curIn) {
            auto clipped = clipSegment(prev, cur, planePoint, planeNormal);
            out.push_back(clipped[1]);
        } else if (!prevIn && curIn) {
            auto clipped = clipSegment(prev, cur, planePoint, planeNormal);
            out.push_back(clipped[0]);
            out.push_back(cur);
        }
    }
    return out;
}

namespace detail {

inline void fillCommon(const Body& bodyA, const Body& bodyB, Manifold& out) noexcept {
    out.restitution = bodyA.restitution * bodyB.restitution;
    out.friction = std::sqrt(bodyA.friction * bodyB.friction);
    out.relVel = bodyB.vel - bodyA.vel;
    // a/b no se tocan: collide() deja ese trabajo al llamante (findContacts
    // asigna los índices del par).
}

[[nodiscard]] inline Vec2 normalizeOr(Vec2 v, Vec2 fallback) noexcept {
    float len = v.length();
    if (len < 1e-8f) {
        float flen = fallback.length();
        return flen < 1e-8f ? Vec2{1.f, 0.f} : fallback / flen;
    }
    return v / len;
}

[[nodiscard]] inline Vec2 closestPointOnSegment(Vec2 p, Vec2 a, Vec2 b, float& tOut) noexcept {
    Vec2 ab = b - a;
    float len2 = ab.length2();
    float t = 0.f;
    if (len2 > 1e-12f) {
        t = std::clamp((p - a).dot(ab) / len2, 0.f, 1.f);
    }
    tOut = t;
    return a + ab * t;
}

[[nodiscard]] inline Vec2 closestPointsSegments(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2, Vec2& out1,
                                                Vec2& out2) noexcept {
    float t1 = 0.f;
    float t2 = 0.f;
    Vec2 d1 = q1 - p1;
    Vec2 d2 = q2 - p2;
    Vec2 r = p1 - p2;
    float a = d1.dot(d1);
    float e = d2.dot(d2);
    float f = d2.dot(r);
    float c = d1.dot(r);
    float b = d1.dot(d2);
    float denom = a * e - b * b;
    float s = 0.f;
    float t = 0.f;
    if (denom > 1e-12f) {
        s = std::clamp((b * f - c * e) / denom, 0.f, 1.f);
        t = e > 1e-12f ? (b * s + f) / e : 0.f;
        t = std::clamp(t, 0.f, 1.f);
        s = a > 1e-12f ? std::clamp((b * t - c) / a, 0.f, 1.f) : 0.f;
    } else if (a > 1e-12f) {
        // Segmentos (casi) paralelos: si se solapan, el punto de contacto
        // estable es el centro del intervalo de solape, no un extremo
        // (un extremo genera torque fantasma en cápsulas/paredes).
        float const tp2 = (p2 - p1).dot(d1) / a;
        float const tq2 = (q2 - p1).dot(d1) / a;
        float const lo = std::max(0.f, std::min(tp2, tq2));
        float const hi = std::min(1.f, std::max(tp2, tq2));
        if (lo <= hi) {
            float const tMid = (lo + hi) * 0.5f;
            out1 = p1 + d1 * tMid;
            float tTmp = 0.f;
            out2 = closestPointOnSegment(out1, p2, q2, tTmp);
            (void)t1;
            (void)t2;
            return out2 - out1;
        }
        float tTmp = 0.f;
        Vec2 c1 = closestPointOnSegment(p1, p2, q2, tTmp);
        Vec2 c2 = closestPointOnSegment(q1, p2, q2, tTmp);
        if ((c1 - p1).length2() < (c2 - q1).length2()) {
            s = 0.f;
            t = tTmp;
            out1 = p1;
            out2 = c1;
            (void)t1;
            (void)t2;
            return out2 - out1;
        }
        s = 1.f;
        out1 = q1;
        out2 = c2;
        (void)t1;
        (void)t2;
        return out2 - out1;
    } else {
        // Segmento degenerado (un punto): lo más cercano del otro.
        float tTmp = 0.f;
        out1 = p1;
        out2 = closestPointOnSegment(p1, p2, q2, tTmp);
        (void)t1;
        (void)t2;
        return out2 - out1;
    }
    t1 = s;
    t2 = t;
    out1 = p1 + d1 * t1;
    out2 = p2 + d2 * t2;
    return out2 - out1;
}

[[nodiscard]] inline bool segmentsIntersect(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4,
                                            Vec2& hit) noexcept {
    Vec2 d1 = p2 - p1;
    Vec2 d2 = p4 - p3;
    float denom = d1.cross(d2);
    if (std::abs(denom) < 1e-12f) {
        float tTmp = 0.f;
        Vec2 cp = closestPointOnSegment(p3, p1, p2, tTmp);
        if ((cp - p3).length2() < 1e-12f) {
            hit = p3;
            return true;
        }
        Vec2 cp2 = closestPointOnSegment(p4, p1, p2, tTmp);
        if ((cp2 - p4).length2() < 1e-12f) {
            hit = p4;
            return true;
        }
        Vec2 cp3 = closestPointOnSegment(p1, p3, p4, tTmp);
        if ((cp3 - p1).length2() < 1e-12f) {
            hit = p1;
            return true;
        }
        Vec2 cp4 = closestPointOnSegment(p2, p3, p4, tTmp);
        if ((cp4 - p2).length2() < 1e-12f) {
            hit = p2;
            return true;
        }
        return false;
    }
    Vec2 r = p3 - p1;
    float t = r.cross(d2) / denom;
    float u = r.cross(d1) / denom;
    if (t < -1e-6f || t > 1.f + 1e-6f || u < -1e-6f || u > 1.f + 1e-6f) {
        return false;
    }
    float tc = std::clamp(t, 0.f, 1.f);
    hit = p1 + d1 * tc;
    return true;
}

// ---------------- Geometry extraction ----------------

struct CircleGeom {
    Vec2 center{};
    float radius = 0.f;
};

struct BoxGeom {
    Vec2 center{};
    Vec2 half{10.f, 10.f};
    float angle = 0.f;
    std::array<Vec2, 4> verts{};
};

struct PolyGeom {
    std::vector<Vec2> verts;
    Vec2 center{};
};

struct CapsuleGeom {
    Vec2 a{};
    Vec2 b{};
    float radius = 0.f;
    Vec2 center{};
};

struct SegmentGeom {
    Vec2 a{};
    Vec2 b{};
    Vec2 center{};
    float radius = 0.f;
};

inline std::array<Vec2, 4> boxVerts(Vec2 centre, Vec2 half, float angle) noexcept {
    std::array<Vec2, 4> local{Vec2{-half.x, -half.y}, Vec2{half.x, -half.y}, Vec2{half.x, half.y},
                              Vec2{-half.x, half.y}};
    std::array<Vec2, 4> world{};
    for (std::size_t i = 0; i < local.size(); ++i) {
        world[i] = local[i].rotated(angle) + centre;
    }
    return world;
}

inline bool extractCircleGeom(const Body& body, CircleGeom& out) noexcept {
    auto const* circle = std::get_if<Circle>(&body.shape);
    if (circle == nullptr) {
        return false;
    }
    out.center = body.pos;
    out.radius = circle->radius;
    return true;
}

inline bool extractBoxGeom(const Body& body, BoxGeom& out) {
    auto const* box = std::get_if<AABB>(&body.shape);
    if (box == nullptr) {
        return false;
    }
    out.half = (box->upper - box->lower) * 0.5f;
    Vec2 localCentre = (box->upper + box->lower) * 0.5f;
    out.angle = body.angle;
    out.center = body.pos + localCentre.rotated(out.angle);
    out.verts = boxVerts(out.center, out.half, out.angle);
    return true;
}

inline bool extractBoxOrAabbGeom(const Body& body, BoxGeom& out) {
    return extractBoxGeom(body, out);
}

inline bool extractPolygonGeom(const Body& body, PolyGeom& out) {
    auto const* poly = std::get_if<ConvexPolygon>(&body.shape);
    if (poly == nullptr) {
        return false;
    }
    for (auto const& v : poly->verts) {
        out.verts.push_back(v.rotated(body.angle) + body.pos);
    }
    if (out.verts.empty()) {
        return false;
    }
    out.center = centroidOf(out.verts);
    return true;
}

inline bool extractCapsuleGeom(const Body& body, CapsuleGeom& out) {
    auto const* capsule = std::get_if<Capsule>(&body.shape);
    if (capsule == nullptr) {
        return false;
    }
    out.a = capsule->capA().rotated(body.angle) + body.pos;
    out.b = capsule->capB().rotated(body.angle) + body.pos;
    out.radius = capsule->radius;
    out.center = (out.a + out.b) * 0.5f;
    return true;
}

inline bool extractSegmentGeom(const Body& body, SegmentGeom& out) {
    auto const* segment = std::get_if<Segment>(&body.shape);
    if (segment == nullptr) {
        return false;
    }
    out.a = segment->a.rotated(body.angle) + body.pos;
    out.b = segment->b.rotated(body.angle) + body.pos;
    out.center = (out.a + out.b) * 0.5f;
    out.radius = segment->radius;
    return true;
}

inline bool extractConvexVerts(const Body& body, std::vector<Vec2>& outVerts) {
    PolyGeom poly{};
    if (extractPolygonGeom(body, poly)) {
        outVerts = poly.verts;
        return true;
    }
    BoxGeom box{};
    if (extractBoxOrAabbGeom(body, box)) {
        outVerts.assign(box.verts.begin(), box.verts.end());
        return true;
    }
    return false;
}

enum class Kind { Circle, Box, Aabb, Polygon, Capsule, Segment, Unknown };

[[nodiscard]] inline Kind kindOf(const Body& body) {
    if (std::holds_alternative<Circle>(body.shape)) {
        return Kind::Circle;
    }
    if (std::holds_alternative<AABB>(body.shape)) {
        return Kind::Box;
    }
    if (std::holds_alternative<ConvexPolygon>(body.shape)) {
        return Kind::Polygon;
    }
    if (std::holds_alternative<Capsule>(body.shape)) {
        return Kind::Capsule;
    }
    if (std::holds_alternative<Segment>(body.shape)) {
        return Kind::Segment;
    }
    return Kind::Unknown;
}

// ---------------- Convex vs convex (SAT + clip) ----------------

[[nodiscard]] inline Vec2 outwardNormal(Vec2 v1, Vec2 v2, Vec2 centroid) noexcept {
    Vec2 n = (v2 - v1).perpendicular();
    float len = n.length();
    if (len < 1e-8f) {
        return Vec2{1.f, 0.f};
    }
    n = n / len;
    if (n.dot(centroid - v1) > 0.f) {
        n = -n;
    }
    return n;
}

inline bool convexVsConvex(std::span<const Vec2> vertsA, std::span<const Vec2> vertsB,
                           const Body& bodyA, const Body& bodyB, Manifold& out) {
    if (vertsA.size() < 3 || vertsB.size() < 3) {
        return false;
    }
    Vec2 centreA = centroidOf(vertsA);
    Vec2 centreB = centroidOf(vertsB);
    float minOverlap = std::numeric_limits<float>::max();
    Vec2 bestAxis{1.f, 0.f};
    bool bestFromA = true;

    auto testAxes = [&](std::span<const Vec2> axesVerts, bool fromA) -> bool {
        Vec2 centre = centroidOf(axesVerts);
        for (std::size_t i = 0; i < axesVerts.size(); ++i) {
            Vec2 v1 = axesVerts[i];
            Vec2 v2 = axesVerts[(i + 1) % axesVerts.size()];
            Vec2 axis = outwardNormal(v1, v2, centre);
            Projection projA = project(vertsA, axis);
            Projection projB = project(vertsB, axis);
            float overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);
            if (overlap < 0.f) {
                return false;
            }
            if (overlap < minOverlap) {
                minOverlap = overlap;
                bestAxis = axis;
                bestFromA = fromA;
            }
        }
        return true;
    };

    if (!testAxes(vertsA, true)) {
        return false;
    }
    if (!testAxes(vertsB, false)) {
        return false;
    }

    Vec2 delta = centreB - centreA;
    if (bestAxis.dot(delta) < 0.f) {
        bestAxis = -bestAxis;
    }
    // A -> B from here on.
    out.normal = bestAxis;
    out.depth = minOverlap;
    fillCommon(bodyA, bodyB, out);

    std::span<const Vec2> refVerts = bestFromA ? vertsA : vertsB;
    std::span<const Vec2> incVerts = bestFromA ? vertsB : vertsA;
    Vec2 refCentre = bestFromA ? centreA : centreB;
    Vec2 refToInc = bestFromA ? bestAxis : -bestAxis;

    std::size_t refIdx = 0;
    float refBest = -std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < refVerts.size(); ++i) {
        Vec2 n = outwardNormal(refVerts[i], refVerts[(i + 1) % refVerts.size()], refCentre);
        float d = n.dot(refToInc);
        if (d > refBest) {
            refBest = d;
            refIdx = i;
        }
    }
    Vec2 refV1 = refVerts[refIdx];
    Vec2 refV2 = refVerts[(refIdx + 1) % refVerts.size()];
    Vec2 refNormal = outwardNormal(refV1, refV2, refCentre);
    Vec2 refTangent = normalizeOr(refV2 - refV1, Vec2{1.f, 0.f});

    auto incident = findIncidentFace(incVerts, refNormal);
    std::array<Vec2, 2> segment{incident[0], incident[1]};
    std::vector<Vec2> clipped = clipSegment(std::span<const Vec2>(segment), refV1, -refTangent);
    if (!clipped.empty()) {
        clipped = clipSegment(std::span<const Vec2>(clipped), refV2, refTangent);
    }

    std::vector<Vec2> contacts;
    float deepest = 0.f;
    for (auto const& p : clipped) {
        float sep = (p - refV1).dot(refNormal);
        if (sep <= 1e-4f) {
            contacts.push_back(p);
            if (-sep > deepest) {
                deepest = -sep;
            }
        }
    }

    if (!contacts.empty()) {
        if (deepest > 0.f) {
            out.depth = deepest;
        }
        out.point = contacts[0];
        if (contacts.size() >= 2) {
            out.point2 = contacts[1];
            out.count = 2;
        } else {
            out.point2 = contacts[0];
            out.count = 1;
        }
    } else {
        out.point = (centreA + centreB) * 0.5f;
        out.point2 = out.point;
        out.count = 1;
        out.depth = minOverlap;
    }
    return true;
}

// ---------------- Circle vs convex polygon ----------------

inline bool circleVsConvex(const Body& circleBody, std::span<const Vec2> polyVerts,
                           const Body& bodyA, const Body& bodyB, Manifold& out, bool circleIsA) {
    if (polyVerts.size() < 3) {
        return false;
    }
    CircleGeom circle{};
    if (!extractCircleGeom(circleBody, circle)) {
        return false;
    }
    Vec2 centre = circle.center;
    float radius = circle.radius;
    Vec2 centroid = centroidOf(polyVerts);

    float maxSep = -std::numeric_limits<float>::max();
    Vec2 maxSepNormal{1.f, 0.f};
    Vec2 maxSepPoint{};
    float minDist2 = std::numeric_limits<float>::max();
    Vec2 closestPoint{};
    for (std::size_t i = 0; i < polyVerts.size(); ++i) {
        Vec2 v1 = polyVerts[i];
        Vec2 v2 = polyVerts[(i + 1) % polyVerts.size()];
        Vec2 n = outwardNormal(v1, v2, centroid);
        float sep = (centre - v1).dot(n);
        float tTmp = 0.f;
        Vec2 cp = closestPointOnSegment(centre, v1, v2, tTmp);
        if (sep > maxSep) {
            maxSep = sep;
            maxSepNormal = n;
            maxSepPoint = cp;
        }
        float d2 = (centre - cp).length2();
        if (d2 < minDist2) {
            minDist2 = d2;
            closestPoint = cp;
        }
    }

    Vec2 canonical{};
    float depth = 0.f;
    Vec2 point{};
    bool inside = maxSep <= 0.f;
    if (inside) {
        canonical = maxSepNormal;
        depth = radius - maxSep;
        point = (centre - canonical * (radius * 0.5f) + maxSepPoint) * 0.5f;
    } else {
        float dist = std::sqrt(minDist2);
        if (dist > radius) {
            return false;
        }
        canonical = dist > 1e-8f ? (centre - closestPoint) / dist : maxSepNormal;
        depth = radius - dist;
        point = (centre - canonical * (radius * 0.5f) + closestPoint) * 0.5f;
    }
    out.normal = circleIsA ? -canonical : canonical;
    out.depth = depth;
    out.point = point;
    out.point2 = point;
    out.count = 1;
    fillCommon(bodyA, bodyB, out);
    return true;
}

// ---------------- Capsule/segment vs convex ----------------
// Canonical normal here is polygon -> capsule (push direction for
// the capsule). Callers orient it to A -> B.

inline bool capsuleVsConvex(const CapsuleGeom& capsule, std::span<const Vec2> polyVerts,
                            const Body& bodyA, const Body& bodyB, Manifold& out, bool capsuleIsA) {
    if (polyVerts.size() < 3) {
        return false;
    }
    Vec2 centroid = centroidOf(polyVerts);
    float radius = capsule.radius;

    // Polygon vertex inside capsule.
    float bestDist = std::numeric_limits<float>::max();
    Vec2 bestVertex{};
    Vec2 bestOnSegment{};
    bool vertexInside = false;
    for (auto const& v : polyVerts) {
        float tTmp = 0.f;
        Vec2 cp = closestPointOnSegment(v, capsule.a, capsule.b, tTmp);
        float d = (v - cp).length();
        if (d < radius && d < bestDist) {
            bestDist = d;
            bestVertex = v;
            bestOnSegment = cp;
            vertexInside = true;
        }
    }
    if (vertexInside) {
        Vec2 canonical =
            normalizeOr(bestOnSegment - bestVertex,
                        normalizeOr(capsule.b - capsule.a, Vec2{1.f, 0.f}).perpendicular());
        out.normal = capsuleIsA ? -canonical : canonical;
        out.depth = radius - bestDist;
        out.point = (bestVertex + bestOnSegment) * 0.5f;
        out.point2 = out.point;
        out.count = 1;
        fillCommon(bodyA, bodyB, out);
        return true;
    }

    // Segment midpoint / endpoints inside polygon.
    auto insideInfo = [&](Vec2 p, float& outSep, Vec2& outNormal) -> bool {
        float maxSep = -std::numeric_limits<float>::max();
        Vec2 maxNormal{1.f, 0.f};
        for (std::size_t i = 0; i < polyVerts.size(); ++i) {
            Vec2 n = outwardNormal(polyVerts[i], polyVerts[(i + 1) % polyVerts.size()], centroid);
            float sep = (p - polyVerts[i]).dot(n);
            if (sep > maxSep) {
                maxSep = sep;
                maxNormal = n;
            }
        }
        outSep = maxSep;
        outNormal = maxNormal;
        return maxSep <= 0.f;
    };

    Vec2 segMid = (capsule.a + capsule.b) * 0.5f;
    float midSep = 0.f;
    Vec2 midNormal{1.f, 0.f};
    bool midInside = insideInfo(segMid, midSep, midNormal);
    if (midInside) {
        out.normal = capsuleIsA ? -midNormal : midNormal;
        out.depth = radius - midSep;
        out.point = segMid - midNormal * (radius * 0.5f);
        out.point2 = out.point;
        out.count = 1;
        fillCommon(bodyA, bodyB, out);
        return true;
    }

    // Outside: closest points between capsule segment and each edge.
    float minDist2 = std::numeric_limits<float>::max();
    Vec2 closestSeg{};
    Vec2 closestPoly{};
    for (std::size_t i = 0; i < polyVerts.size(); ++i) {
        Vec2 cpSeg{};
        Vec2 cpPoly{};
        Vec2 diff = closestPointsSegments(capsule.a, capsule.b, polyVerts[i],
                                          polyVerts[(i + 1) % polyVerts.size()], cpSeg, cpPoly);
        float d2 = diff.length2();
        if (d2 < minDist2) {
            minDist2 = d2;
            closestSeg = cpSeg;
            closestPoly = cpPoly;
        }
    }
    float dist = std::sqrt(minDist2);
    if (dist > radius) {
        return false;
    }
    Vec2 canonical = dist > 1e-8f ? (closestSeg - closestPoly) / dist : midNormal;
    out.normal = capsuleIsA ? -canonical : canonical;
    out.depth = radius - dist;
    out.point = (closestSeg + closestPoly) * 0.5f;
    out.point2 = out.point;
    out.count = 1;
    fillCommon(bodyA, bodyB, out);
    return true;
}

} // namespace detail

// ------------------------------------------------------------------
// Pairwise functions. All take (A, B, out) in any shape order and
// write out.normal as A -> B.
// ------------------------------------------------------------------

[[nodiscard]] inline bool circleCircle(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CircleGeom geomA{};
    detail::CircleGeom geomB{};
    if (!detail::extractCircleGeom(bodyA, geomA) || !detail::extractCircleGeom(bodyB, geomB)) {
        return false;
    }
    Vec2 delta = geomB.center - geomA.center;
    float rSum = geomA.radius + geomB.radius;
    float dist2 = delta.length2();
    if (dist2 >= rSum * rSum) {
        return false;
    }
    float dist = std::sqrt(dist2);
    Vec2 n = dist > 1e-8f ? delta / dist : Vec2{1.f, 0.f};
    out.normal = n;
    out.depth = rSum - dist;
    out.point = geomA.center + n * (geomA.radius - out.depth * 0.5f);
    out.point2 = out.point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[nodiscard]] inline bool circleAABB(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CircleGeom geomA{};
    detail::CircleGeom geomB{};
    bool aCircle = detail::extractCircleGeom(bodyA, geomA);
    bool bCircle = detail::extractCircleGeom(bodyB, geomB);
    if (aCircle == bCircle) {
        return false;
    }
    detail::CircleGeom circleGeom = aCircle ? geomA : geomB;
    const Body& boxBody = aCircle ? bodyB : bodyA;
    detail::BoxGeom boxGeom{};
    if (!detail::extractBoxOrAabbGeom(boxBody, boxGeom)) {
        return false;
    }
    Vec2 local = (circleGeom.center - boxGeom.center).rotated(-boxGeom.angle);
    Vec2 clampedGeo{std::clamp(local.x, -boxGeom.half.x, boxGeom.half.x),
                    std::clamp(local.y, -boxGeom.half.y, boxGeom.half.y)};
    Vec2 diffLocal = local - clampedGeo;
    float dist2 = diffLocal.length2();
    if (dist2 > circleGeom.radius * circleGeom.radius) {
        return false;
    }
    Vec2 canonicalLocal{};
    float depth = 0.f;
    if (dist2 > 1e-12f) {
        float dist = std::sqrt(dist2);
        canonicalLocal = diffLocal / dist;
        depth = circleGeom.radius - dist;
    } else {
        float dx = boxGeom.half.x - std::abs(local.x);
        float dy = boxGeom.half.y - std::abs(local.y);
        if (dx < dy) {
            canonicalLocal = Vec2{local.x >= 0.f ? 1.f : -1.f, 0.f};
            depth = circleGeom.radius + dx;
        } else {
            canonicalLocal = Vec2{0.f, local.y >= 0.f ? 1.f : -1.f};
            depth = circleGeom.radius + dy;
        }
    }
    Vec2 canonical = canonicalLocal.rotated(boxGeom.angle);
    Vec2 closestWorld = clampedGeo.rotated(boxGeom.angle) + boxGeom.center;
    out.normal = aCircle ? -canonical : canonical;
    out.depth = depth;
    out.point = (circleGeom.center - canonical * (circleGeom.radius * 0.5f) + closestWorld) * 0.5f;
    out.point2 = out.point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[deprecated("Use circleAABB")]] inline bool circleBox(Body& bodyA, Body& bodyB, Manifold& out) {
    return circleAABB(bodyA, bodyB, out);
}

[[nodiscard]] inline bool circlePolygon(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CircleGeom circleGeom{};
    bool aCircle = detail::extractCircleGeom(bodyA, circleGeom);
    bool bCircle = false;
    if (!aCircle) {
        bCircle = detail::extractCircleGeom(bodyB, circleGeom);
    }
    if (aCircle == bCircle) {
        return false;
    }
    const Body& polyBody = aCircle ? bodyB : bodyA;
    detail::PolyGeom poly{};
    if (!detail::extractPolygonGeom(polyBody, poly)) {
        return false;
    }
    return detail::circleVsConvex(aCircle ? bodyA : bodyB, std::span<const Vec2>(poly.verts), bodyA,
                                  bodyB, out, aCircle);
}

[[nodiscard]] inline bool circleCapsule(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CircleGeom circleGeom{};
    bool aCircle = detail::extractCircleGeom(bodyA, circleGeom);
    bool bCircle = false;
    if (!aCircle) {
        bCircle = detail::extractCircleGeom(bodyB, circleGeom);
    }
    if (aCircle == bCircle) {
        return false;
    }
    const Body& capsuleBody = aCircle ? bodyB : bodyA;
    detail::CapsuleGeom capsule{};
    if (!detail::extractCapsuleGeom(capsuleBody, capsule)) {
        return false;
    }
    float tTmp = 0.f;
    Vec2 closest = detail::closestPointOnSegment(circleGeom.center, capsule.a, capsule.b, tTmp);
    Vec2 delta = circleGeom.center - closest;
    float dist = delta.length();
    float rSum = circleGeom.radius + capsule.radius;
    if (dist > rSum) {
        return false;
    }
    Vec2 canonical = detail::normalizeOr(
        delta, detail::normalizeOr(capsule.b - capsule.a, Vec2{1.f, 0.f}).perpendicular());
    out.normal = aCircle ? -canonical : canonical;
    out.depth = rSum - dist;
    out.point = (closest + circleGeom.center) * 0.5f;
    out.point2 = out.point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[nodiscard]] inline bool circleSegment(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CircleGeom circleGeom{};
    bool aCircle = detail::extractCircleGeom(bodyA, circleGeom);
    bool bCircle = false;
    if (!aCircle) {
        bCircle = detail::extractCircleGeom(bodyB, circleGeom);
    }
    if (aCircle == bCircle) {
        return false;
    }
    const Body& segmentBody = aCircle ? bodyB : bodyA;
    detail::SegmentGeom segment{};
    if (!detail::extractSegmentGeom(segmentBody, segment)) {
        return false;
    }
    float tTmp = 0.f;
    Vec2 closest = detail::closestPointOnSegment(circleGeom.center, segment.a, segment.b, tTmp);
    Vec2 delta = circleGeom.center - closest;
    float dist = delta.length();
    float minDist = circleGeom.radius + segment.radius;
    if (dist > minDist) {
        return false;
    }
    Vec2 canonical = detail::normalizeOr(
        delta, detail::normalizeOr(segment.b - segment.a, Vec2{1.f, 0.f}).perpendicular());
    out.normal = aCircle ? -canonical : canonical;
    out.depth = minDist - dist;
    out.point = (closest + circleGeom.center) * 0.5f;
    out.point2 = out.point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[nodiscard]] inline bool capsuleCapsule(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CapsuleGeom capsuleA{};
    detail::CapsuleGeom capsuleB{};
    if (!detail::extractCapsuleGeom(bodyA, capsuleA) ||
        !detail::extractCapsuleGeom(bodyB, capsuleB)) {
        return false;
    }
    Vec2 closestA{};
    Vec2 closestB{};
    Vec2 diff = detail::closestPointsSegments(capsuleA.a, capsuleA.b, capsuleB.a, capsuleB.b,
                                              closestA, closestB);
    float dist = diff.length();
    float rSum = capsuleA.radius + capsuleB.radius;
    if (dist > rSum) {
        return false;
    }
    Vec2 fallback = detail::normalizeOr(capsuleA.b - capsuleA.a, Vec2{1.f, 0.f}).perpendicular();
    out.normal = detail::normalizeOr(diff, fallback);
    out.depth = rSum - dist;
    out.point = (closestA + closestB) * 0.5f;
    out.point2 = out.point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[nodiscard]] inline bool capsulePolygon(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CapsuleGeom capsule{};
    bool aCapsule = detail::extractCapsuleGeom(bodyA, capsule);
    bool bCapsule = false;
    if (!aCapsule) {
        bCapsule = detail::extractCapsuleGeom(bodyB, capsule);
    }
    if (aCapsule == bCapsule) {
        return false;
    }
    const Body& polyBody = aCapsule ? bodyB : bodyA;
    detail::PolyGeom poly{};
    if (detail::extractPolygonGeom(polyBody, poly)) {
        return detail::capsuleVsConvex(capsule, std::span<const Vec2>(poly.verts), bodyA, bodyB,
                                       out, aCapsule);
    }
    detail::BoxGeom box{};
    if (!detail::extractBoxOrAabbGeom(polyBody, box)) {
        return false;
    }
    return detail::capsuleVsConvex(capsule, std::span<const Vec2>(box.verts), bodyA, bodyB, out,
                                   aCapsule);
}

[[nodiscard]] inline bool capsuleBox(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CapsuleGeom capsule{};
    bool aCapsule = detail::extractCapsuleGeom(bodyA, capsule);
    bool bCapsule = false;
    if (!aCapsule) {
        bCapsule = detail::extractCapsuleGeom(bodyB, capsule);
    }
    if (aCapsule == bCapsule) {
        return false;
    }
    const Body& boxBody = aCapsule ? bodyB : bodyA;
    detail::BoxGeom box{};
    if (!detail::extractBoxOrAabbGeom(boxBody, box)) {
        return false;
    }
    return detail::capsuleVsConvex(capsule, std::span<const Vec2>(box.verts), bodyA, bodyB, out,
                                   aCapsule);
}

[[nodiscard]] inline bool capsuleSegment(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::CapsuleGeom capsule{};
    detail::SegmentGeom segment{};
    bool aCapsule = detail::extractCapsuleGeom(bodyA, capsule);
    bool bCapsule = false;
    bool aSegment = false;
    bool bSegment = false;
    if (aCapsule) {
        bSegment = detail::extractSegmentGeom(bodyB, segment);
    } else {
        bCapsule = detail::extractCapsuleGeom(bodyB, capsule);
        if (bCapsule) {
            aSegment = detail::extractSegmentGeom(bodyA, segment);
        }
    }
    if (!((aCapsule && bSegment) || (bCapsule && aSegment))) {
        return false;
    }
    Vec2 closestCap{};
    Vec2 closestSeg{};
    Vec2 diff = detail::closestPointsSegments(capsule.a, capsule.b, segment.a, segment.b,
                                              closestCap, closestSeg);
    float dist = diff.length();
    float rSum = capsule.radius + segment.radius;
    if (dist > rSum) {
        return false;
    }
    Vec2 fallback = detail::normalizeOr(capsule.b - capsule.a, Vec2{1.f, 0.f}).perpendicular();
    Vec2 canonical = detail::normalizeOr(closestSeg - closestCap, fallback);
    out.normal = aCapsule ? canonical : -canonical;
    out.depth = rSum - dist;
    out.point = (closestCap + closestSeg) * 0.5f;
    out.point2 = out.point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[nodiscard]] inline bool segmentSegment(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::SegmentGeom segmentA{};
    detail::SegmentGeom segmentB{};
    if (!detail::extractSegmentGeom(bodyA, segmentA) ||
        !detail::extractSegmentGeom(bodyB, segmentB)) {
        return false;
    }
    Vec2 hit{};
    bool hitFlag = detail::segmentsIntersect(segmentA.a, segmentA.b, segmentB.a, segmentB.b, hit);
    Vec2 closestA{};
    Vec2 closestB{};
    Vec2 diff = detail::closestPointsSegments(segmentA.a, segmentA.b, segmentB.a, segmentB.b,
                                              closestA, closestB);
    float dist = diff.length();
    float const rSum = segmentA.radius + segmentB.radius;
    if (rSum > 0.f) {
        // Segmentos gruesos: se comportan como cápsulas 1D (solapan si dist <= rA+rB).
        if (dist > rSum && !hitFlag) {
            return false;
        }
        float const depth = dist <= rSum ? rSum - dist : 0.f;
        Vec2 const point = dist > 1e-8f ? (closestA + closestB) * 0.5f : (hitFlag ? hit : closestA);
        Vec2 const fallback =
            detail::normalizeOr(segmentA.b - segmentA.a, Vec2{1.f, 0.f}).perpendicular();
        Vec2 const towardB = segmentB.center - segmentA.center;
        Vec2 const fallbackB = fallback.dot(towardB) < 0.f ? -fallback : fallback;
        out.normal = detail::normalizeOr(diff, fallbackB);
        out.depth = depth;
        out.point = point;
        out.point2 = point;
        out.count = 1;
        detail::fillCommon(bodyA, bodyB, out);
        return true;
    }
    if (!hitFlag && dist > 1e-6f) {
        return false;
    }
    Vec2 point = hitFlag ? hit : (closestA + closestB) * 0.5f;
    Vec2 fallback = detail::normalizeOr(segmentA.b - segmentA.a, Vec2{1.f, 0.f}).perpendicular();
    Vec2 towardB = segmentB.center - segmentA.center;
    if (fallback.dot(towardB) < 0.f) {
        fallback = -fallback;
    }
    out.normal = detail::normalizeOr(diff, fallback);
    out.depth = 0.f;
    out.point = point;
    out.point2 = point;
    out.count = 1;
    detail::fillCommon(bodyA, bodyB, out);
    return true;
}

[[nodiscard]] inline bool segmentPolygon(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::SegmentGeom segment{};
    bool aSegment = detail::extractSegmentGeom(bodyA, segment);
    bool bSegment = false;
    if (!aSegment) {
        bSegment = detail::extractSegmentGeom(bodyB, segment);
    }
    if (aSegment == bSegment) {
        return false;
    }
    const Body& polyBody = aSegment ? bodyB : bodyA;
    detail::CapsuleGeom capsule{};
    capsule.a = segment.a;
    capsule.b = segment.b;
    capsule.radius = segment.radius;
    capsule.center = segment.center;
    detail::PolyGeom poly{};
    if (detail::extractPolygonGeom(polyBody, poly)) {
        return detail::capsuleVsConvex(capsule, std::span<const Vec2>(poly.verts), bodyA, bodyB,
                                       out, aSegment);
    }
    detail::BoxGeom box{};
    if (!detail::extractBoxOrAabbGeom(polyBody, box)) {
        return false;
    }
    return detail::capsuleVsConvex(capsule, std::span<const Vec2>(box.verts), bodyA, bodyB, out,
                                   aSegment);
}

[[nodiscard]] inline bool segmentBox(Body& bodyA, Body& bodyB, Manifold& out) {
    return segmentPolygon(bodyA, bodyB, out);
}

[[nodiscard]] inline bool boxBox(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::BoxGeom boxA{};
    detail::BoxGeom boxB{};
    if (!detail::extractBoxOrAabbGeom(bodyA, boxA) || !detail::extractBoxOrAabbGeom(bodyB, boxB)) {
        return false;
    }
    return detail::convexVsConvex(std::span<const Vec2>(boxA.verts),
                                  std::span<const Vec2>(boxB.verts), bodyA, bodyB, out);
}

[[nodiscard]] inline bool polygonPolygon(Body& bodyA, Body& bodyB, Manifold& out) {
    detail::PolyGeom polyA{};
    detail::PolyGeom polyB{};
    detail::BoxGeom boxA{};
    detail::BoxGeom boxB{};
    std::span<const Vec2> spanA;
    std::span<const Vec2> spanB;
    if (detail::extractPolygonGeom(bodyA, polyA)) {
        spanA = std::span<const Vec2>(polyA.verts);
    } else if (detail::extractBoxOrAabbGeom(bodyA, boxA)) {
        spanA = std::span<const Vec2>(boxA.verts);
    } else {
        return false;
    }
    if (detail::extractPolygonGeom(bodyB, polyB)) {
        spanB = std::span<const Vec2>(polyB.verts);
    } else if (detail::extractBoxOrAabbGeom(bodyB, boxB)) {
        spanB = std::span<const Vec2>(boxB.verts);
    } else {
        return false;
    }
    return detail::convexVsConvex(spanA, spanB, bodyA, bodyB, out);
}

// ------------------------------------------------------------------
// Dispatcher: std::visit / holds_alternative based, normal = A -> B.
// No normal flipping here: each callee already returns A -> B.
// ------------------------------------------------------------------

[[nodiscard]] inline bool collide(Body& bodyA, Body& bodyB, Manifold& out) {
    // Shape dispatch is via detail::kindOf (holds_alternative over
    // Body::shape classifying Circle/AABB/ConvexPolygon/Capsule/Segment).
    detail::Kind kindA = detail::kindOf(bodyA);
    detail::Kind kindB = detail::kindOf(bodyB);

    if (kindA == detail::Kind::Circle && kindB == detail::Kind::Circle) {
        return circleCircle(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Circle &&
         (kindB == detail::Kind::Box || kindB == detail::Kind::Aabb)) ||
        (kindB == detail::Kind::Circle &&
         (kindA == detail::Kind::Box || kindA == detail::Kind::Aabb))) {
        return circleAABB(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Circle && kindB == detail::Kind::Polygon) ||
        (kindB == detail::Kind::Circle && kindA == detail::Kind::Polygon)) {
        return circlePolygon(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Circle && kindB == detail::Kind::Capsule) ||
        (kindB == detail::Kind::Circle && kindA == detail::Kind::Capsule)) {
        return circleCapsule(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Circle && kindB == detail::Kind::Segment) ||
        (kindB == detail::Kind::Circle && kindA == detail::Kind::Segment)) {
        return circleSegment(bodyA, bodyB, out);
    }
    if (kindA == detail::Kind::Capsule && kindB == detail::Kind::Capsule) {
        return capsuleCapsule(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Capsule && kindB == detail::Kind::Polygon) ||
        (kindB == detail::Kind::Capsule && kindA == detail::Kind::Polygon)) {
        return capsulePolygon(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Capsule &&
         (kindB == detail::Kind::Box || kindB == detail::Kind::Aabb)) ||
        (kindB == detail::Kind::Capsule &&
         (kindA == detail::Kind::Box || kindA == detail::Kind::Aabb))) {
        return capsuleBox(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Capsule && kindB == detail::Kind::Segment) ||
        (kindB == detail::Kind::Capsule && kindA == detail::Kind::Segment)) {
        return capsuleSegment(bodyA, bodyB, out);
    }
    if (kindA == detail::Kind::Segment && kindB == detail::Kind::Segment) {
        return segmentSegment(bodyA, bodyB, out);
    }
    if ((kindA == detail::Kind::Segment &&
         (kindB == detail::Kind::Polygon || kindB == detail::Kind::Box ||
          kindB == detail::Kind::Aabb)) ||
        (kindB == detail::Kind::Segment &&
         (kindA == detail::Kind::Polygon || kindA == detail::Kind::Box ||
          kindA == detail::Kind::Aabb))) {
        return segmentPolygon(bodyA, bodyB, out);
    }
    bool aConvex =
        kindA == detail::Kind::Box || kindA == detail::Kind::Aabb || kindA == detail::Kind::Polygon;
    bool bConvex =
        kindB == detail::Kind::Box || kindB == detail::Kind::Aabb || kindB == detail::Kind::Polygon;
    if (aConvex && bConvex) {
        if ((kindA == detail::Kind::Box || kindA == detail::Kind::Aabb) &&
            (kindB == detail::Kind::Box || kindB == detail::Kind::Aabb)) {
            return boxBox(bodyA, bodyB, out);
        }
        return polygonPolygon(bodyA, bodyB, out);
    }
    return false;
}

} // namespace phys2d
