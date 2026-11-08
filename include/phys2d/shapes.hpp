#pragma once
#include "phys2d/mat2.hpp"
#include "phys2d/vec2.hpp"
#include <algorithm>
#include <cmath>
#include <type_traits>
#include <variant>
#include <vector>

namespace phys2d {

// phys2d/shapes.hpp — formas en espacio LOCAL respecto a Body::pos/angle.
//   BodyType vive aqui (no en body.hpp) porque massProperties() lo necesita sin
//   include circular; body.hpp debe incluir shapes.hpp y no redefinirlo.
//   Box es alias de AABB: la rotacion es Body::angle, no se guarda en la forma.
//   Params son aggregates planos (C++23 exige todo-designado o todo-posicional
//   por lista, NO se puede mezclar {{100,50}, .radius=20}):
//     w.addDynamicCircle(CircleParams{{100.f,50.f}, 20.f});            // posicional
//     w.addDynamicCircle({.pos={100.f,50.f}, .radius=20.f, .vel={40.f,0.f}});
//     w.addDynamicBox({.pos={400.f,40.f}, .half={40.f,40.f}, .angle=0.3f});
//     w.addDynamicCapsule({.pos={200.f,60.f}, .radius=8.f, .length=24.f});
//     w.addDynamicPolygon({.pos={500.f,80.f}, .verts={{-20.f,-10.f},{20.f,-10.f},{0.f,20.f}}});
//   World mapea addDynamicCircle(CircleParams) -> círculo validado por addCircle,
//   etc.; addStatic*/addKinematic* fuerzan BodyType sobre el campo type.
//   Para suelo: BoxParams::fromLowerUpper({0,590},{1200,610}) o fromSize().
inline constexpr float kPi = 3.14159265358979323846f;

// Tipo de cuerpo fisico.
enum class BodyType : unsigned char { Static, Kinematic, Dynamic };

// Masa/inercia ya calculadas (centro en espacio local).
struct MassProps {
    Vec2 center{};
    float mass = 0.f;
    float invMass = 0.f;
    float inertia = 0.f;
    float invInertia = 0.f;
    [[nodiscard]] constexpr bool isDynamic() const noexcept {
        return invMass > 0.f;
    }
};

// Caja axis-aligned en espacio local (rotacion = Body::angle).
struct AABB {
    Vec2 lower{};
    Vec2 upper{};

    [[nodiscard]] constexpr Vec2 center() const noexcept {
        return (lower + upper) * 0.5f;
    }
    [[nodiscard]] constexpr Vec2 half() const noexcept {
        return (upper - lower) * 0.5f;
    }
    [[nodiscard]] constexpr Vec2 size() const noexcept {
        return upper - lower;
    }
    [[nodiscard]] constexpr float width() const noexcept {
        return upper.x - lower.x;
    }
    [[nodiscard]] constexpr float height() const noexcept {
        return upper.y - lower.y;
    }
    [[nodiscard]] constexpr float area() const noexcept {
        return width() * height();
    }
    [[nodiscard]] constexpr float perimeter() const noexcept {
        return 2.f * (width() + height());
    }
    [[nodiscard]] constexpr bool isValid() const noexcept {
        return lower.x <= upper.x && lower.y <= upper.y;
    }
    [[nodiscard]] constexpr bool contains(Vec2 point) const noexcept {
        return point.x >= lower.x && point.x <= upper.x && point.y >= lower.y && point.y <= upper.y;
    }
    [[nodiscard]] constexpr bool contains(AABB other) const noexcept {
        return contains(other.lower) && contains(other.upper);
    }
    [[nodiscard]] constexpr bool intersects(AABB other) const noexcept {
        return other.lower.x <= upper.x && lower.x <= other.upper.x && other.lower.y <= upper.y &&
               lower.y <= other.upper.y;
    }
    [[nodiscard]] constexpr Vec2 closestPoint(Vec2 point) const noexcept {
        return point.clamp(lower, upper);
    }
    [[nodiscard]] constexpr AABB expanded(float margin) const noexcept {
        Vec2 m{margin, margin};
        return {lower - m, upper + m};
    }
    [[nodiscard]] constexpr AABB combined(AABB other) const noexcept {
        return {min(lower, other.lower), max(upper, other.upper)};
    }
    [[nodiscard]] constexpr bool testPoint(Vec2 point) const noexcept {
        return contains(point);
    }

    [[nodiscard]] static constexpr AABB fromCenterHalf(Vec2 c, Vec2 h) noexcept {
        return {c - h, c + h};
    }
    [[nodiscard]] static constexpr AABB fromCenterSize(Vec2 c, Vec2 s) noexcept {
        return {c - s * 0.5f, c + s * 0.5f};
    }
    [[nodiscard]] static constexpr AABB fromLowerUpper(Vec2 lo, Vec2 hi) noexcept {
        return {lo, hi};
    }
};

// Alias historico: Box ya no guarda angle, la rotacion esta en Body.
using Box = AABB;

// Circulo centrado en el origen local.
struct Circle {
    float radius = 10.f;
    [[nodiscard]] constexpr float area() const noexcept {
        return kPi * radius * radius;
    }
    [[nodiscard]] constexpr float perimeter() const noexcept {
        return 2.f * kPi * radius;
    }
    [[nodiscard]] constexpr bool testPoint(Vec2 point) const noexcept {
        return point.length2() <= radius * radius;
    }
    [[nodiscard]] Vec2 closestPoint(Vec2 point) const noexcept {
        return point.length2() <= radius * radius ? point : point.normalized() * radius;
    }
    [[nodiscard]] constexpr AABB aabb() const noexcept {
        Vec2 r{radius, radius};
        return {-r, r};
    }
};

// Poligono convexo en espacio local (verts antihorarios).
struct ConvexPolygon {
    std::vector<Vec2> verts;

    [[nodiscard]] std::size_t size() const noexcept {
        return verts.size();
    }
    [[nodiscard]] bool empty() const noexcept {
        return verts.empty();
    }
    [[nodiscard]] float signedArea() const noexcept {
        float sum = 0.f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            sum += a.cross(b);
        }
        return sum * 0.5f;
    }
    [[nodiscard]] float area() const noexcept {
        float v = signedArea();
        return v >= 0.f ? v : -v;
    }
    [[nodiscard]] float perimeter() const noexcept {
        float sum = 0.f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            sum += (b - a).length();
        }
        return sum;
    }
    [[nodiscard]] bool isConvex() const noexcept {
        if (verts.size() < 3) {
            return false;
        }
        float sign = 0.f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            Vec2 c = verts[(i + 2) % verts.size()];
            float cr = (b - a).cross(c - b);
            if (cr > 1e-8f) {
                if (sign < 0.f) {
                    return false;
                }
                sign = 1.f;
            } else if (cr < -1e-8f) {
                if (sign > 0.f) {
                    return false;
                }
                sign = -1.f;
            }
        }
        return true;
    }
    [[nodiscard]] bool isValid() const noexcept {
        return verts.size() >= 3 && area() > 1e-8f && isConvex();
    }
    [[nodiscard]] Vec2 centroid() const noexcept {
        if (verts.empty()) {
            return {};
        }
        float signedA = signedArea();
        if (signedA >= 1e-8f || signedA <= -1e-8f) {
            Vec2 sum{};
            for (std::size_t i = 0; i < verts.size(); ++i) {
                Vec2 a = verts[i];
                Vec2 b = verts[(i + 1) % verts.size()];
                float cr = a.cross(b);
                sum += (a + b) * cr;
            }
            return sum / (6.f * signedA);
        }
        Vec2 avg{};
        for (auto v : verts) {
            avg += v;
        }
        return avg / static_cast<float>(verts.size());
    }
    [[nodiscard]] MassProps massProps(float density) const noexcept {
        MassProps props;
        props.center = centroid();
        if (verts.size() < 3 || density <= 0.f) {
            return props;
        }
        float signedA = signedArea();
        float absArea = signedA >= 0.f ? signedA : -signedA;
        if (absArea <= 1e-8f) {
            return props;
        }
        float inertiaSum = 0.f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            float cr = a.cross(b);
            inertiaSum += cr * (a.dot(a) + a.dot(b) + b.dot(b));
        }
        float m = density * absArea;
        float inertiaOrigin = (density / 12.f) * inertiaSum;
        if (inertiaOrigin < 0.f) {
            inertiaOrigin = -inertiaOrigin;
        }
        float inertia = inertiaOrigin - m * props.center.length2();
        if (inertia < 0.f) {
            inertia = 0.f;
        }
        props.mass = m;
        props.invMass = 1.f / m;
        props.inertia = inertia;
        props.invInertia = inertia > 1e-8f ? 1.f / inertia : 0.f;
        return props;
    }
    [[nodiscard]] std::vector<Vec2> transform(Vec2 pos, float angle) const {
        Mat2 m = Mat2::rotation(angle);
        std::vector<Vec2> out;
        out.reserve(verts.size());
        for (auto v : verts) {
            out.push_back(m * v + pos);
        }
        return out;
    }
    [[nodiscard]] bool testPoint(Vec2 point) const noexcept {
        if (verts.size() < 3) {
            return false;
        }
        float sign = signedArea() >= 0.f ? 1.f : -1.f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            float cr = (b - a).cross(point - a);
            if (sign * cr < -1e-6f) {
                return false;
            }
        }
        return true;
    }
    [[nodiscard]] Vec2 closestPoint(Vec2 point) const {
        if (verts.empty()) {
            return {};
        }
        if (testPoint(point)) {
            return point;
        }
        Vec2 best = verts[0];
        float bestDist2 = (best - point).length2();
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            Vec2 ab = b - a;
            float segLen2 = ab.length2();
            float t = segLen2 > 1e-12f ? std::clamp((point - a).dot(ab) / segLen2, 0.f, 1.f) : 0.f;
            Vec2 cand = a + ab * t;
            float d2 = (cand - point).length2();
            if (d2 < bestDist2) {
                bestDist2 = d2;
                best = cand;
            }
        }
        return best;
    }
    [[nodiscard]] AABB aabb(float angle) const {
        return aabb(Vec2{}, angle);
    }
    [[nodiscard]] AABB aabb(Vec2 pos, float angle) const {
        if (verts.empty()) {
            return {};
        }
        Mat2 m = Mat2::rotation(angle);
        Vec2 lo = m * verts[0] + pos;
        Vec2 hi = lo;
        for (std::size_t i = 1; i < verts.size(); ++i) {
            Vec2 p = m * verts[i] + pos;
            lo = min(lo, p);
            hi = max(hi, p);
        }
        return {lo, hi};
    }
    [[nodiscard]] std::vector<Vec2> edgeNormals() const {
        std::vector<Vec2> out;
        out.reserve(verts.size());
        float sign = signedArea() >= 0.f ? 1.f : -1.f;
        for (std::size_t i = 0; i < verts.size(); ++i) {
            Vec2 a = verts[i];
            Vec2 b = verts[(i + 1) % verts.size()];
            Vec2 n = (b - a).perpendicular().normalized();
            out.push_back(n * (-sign));
        }
        return out;
    }
};

// Capsula: dir normalizado, length = altura total incluyendo los tapones.
struct Capsule {
    float radius = 10.f;
    float length = 20.f;
    Vec2 dir{0.f, 1.f};

    [[nodiscard]] constexpr float midLength() const noexcept {
        return length > 2.f * radius ? length - 2.f * radius : 0.f;
    }
    [[nodiscard]] constexpr Vec2 capA() const noexcept {
        return dir * (midLength() * 0.5f);
    }
    [[nodiscard]] constexpr Vec2 capB() const noexcept {
        return dir * (-midLength() * 0.5f);
    }
    [[nodiscard]] constexpr Vec2 a() const noexcept {
        return capA();
    }
    [[nodiscard]] constexpr Vec2 b() const noexcept {
        return capB();
    }
    [[nodiscard]] constexpr float area() const noexcept {
        return kPi * radius * radius + 2.f * radius * midLength();
    }
    [[nodiscard]] Vec2 closestAxisPoint(Vec2 point) const {
        Vec2 ca = capA();
        Vec2 cb = capB();
        Vec2 ab = cb - ca;
        float segLen2 = ab.length2();
        if (segLen2 <= 1e-12f) {
            return ca;
        }
        float t = std::clamp((point - ca).dot(ab) / segLen2, 0.f, 1.f);
        return ca + ab * t;
    }
    [[nodiscard]] Vec2 closestPoint(Vec2 point) const {
        Vec2 axisPt = closestAxisPoint(point);
        Vec2 diff = point - axisPt;
        float d2 = diff.length2();
        if (d2 <= radius * radius) {
            return point;
        }
        float d = std::sqrt(d2);
        return d > 1e-8f ? axisPt + diff / d * radius : point;
    }
    [[nodiscard]] bool testPoint(Vec2 point) const {
        return (point - closestAxisPoint(point)).length2() <= radius * radius;
    }
    [[nodiscard]] AABB aabb(float angle) const {
        Vec2 ca = capA().rotated(angle);
        Vec2 cb = capB().rotated(angle);
        Vec2 ext{radius, radius};
        return {min(ca, cb) - ext, max(ca, cb) + ext};
    }
    [[nodiscard]] AABB aabb(Vec2 pos, float angle) const {
        AABB local = aabb(angle);
        return {local.lower + pos, local.upper + pos};
    }
    [[nodiscard]] static Capsule fromSegment(Vec2 segA, Vec2 segB, float r) noexcept {
        Vec2 delta = segB - segA;
        float len = delta.length();
        Vec2 d = len > 1e-8f ? delta / len : Vec2{0.f, 1.f};
        return Capsule{r, len + 2.f * r, d};
    }
};

// Segmento local de a a b con grosor radius (radius 0 = linea).
struct Segment {
    Vec2 a{};
    Vec2 b{100.f, 0.f};
    float radius = 0.f;

    [[nodiscard]] constexpr Vec2 delta() const noexcept {
        return b - a;
    }
    [[nodiscard]] float length() const noexcept {
        return delta().length();
    }
    [[nodiscard]] Vec2 direction() const noexcept {
        Vec2 d = delta();
        float len = d.length();
        return len > 1e-8f ? d / len : Vec2{1.f, 0.f};
    }
    [[nodiscard]] Vec2 normal() const noexcept {
        return direction().perpendicular();
    }
    [[nodiscard]] Vec2 closestPoint(Vec2 point) const {
        Vec2 ab = delta();
        float segLen2 = ab.length2();
        if (segLen2 <= 1e-12f) {
            return a;
        }
        float t = std::clamp((point - a).dot(ab) / segLen2, 0.f, 1.f);
        return a + ab * t;
    }
    [[nodiscard]] Vec2 closestAxisPoint(Vec2 point) const {
        return closestPoint(point);
    }
    [[nodiscard]] Vec2 closestSurfacePoint(Vec2 point) const {
        Vec2 axisPt = closestPoint(point);
        if (radius <= 0.f) {
            return axisPt;
        }
        Vec2 diff = point - axisPt;
        float d2 = diff.length2();
        if (d2 <= radius * radius) {
            return point;
        }
        float d = std::sqrt(d2);
        return d > 1e-8f ? axisPt + diff / d * radius : axisPt + Vec2{radius, 0.f};
    }
    [[nodiscard]] bool testPoint(Vec2 point) const {
        float r = radius > 0.f ? radius : 1e-4f;
        return (point - closestPoint(point)).length2() <= r * r;
    }
    [[nodiscard]] AABB aabb(float angle) const {
        Vec2 ra = a.rotated(angle);
        Vec2 rb = b.rotated(angle);
        Vec2 ext{radius, radius};
        return {min(ra, rb) - ext, max(ra, rb) + ext};
    }
    [[nodiscard]] AABB aabb(Vec2 pos, float angle) const {
        AABB local = aabb(angle);
        return {local.lower + pos, local.upper + pos};
    }
};

// Variante de formas locales (Box = AABB).
using Shape = std::variant<Circle, AABB, ConvexPolygon, Capsule, Segment>;

// Params genericos de cuerpo.
struct BodyParams {
    Vec2 pos{};
    Vec2 vel{};
    float angle = 0.f;
    float angVel = 0.f;
    float mass = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    BodyType type = BodyType::Dynamic;
};

// Params de circulo: CircleParams{{100.f,50.f}, 20.f} o {.pos=..., .radius=...}.
struct CircleParams {
    Vec2 pos{};
    float radius = 10.f;
    Vec2 vel{};
    float angle = 0.f;
    float angVel = 0.f;
    float mass = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    BodyType type = BodyType::Dynamic;
};

// Params de caja (2o campo = half): {.pos=..., .half=..., .angle=...}.
struct BoxParams {
    Vec2 pos{};
    Vec2 half{10.f, 10.f};
    Vec2 vel{};
    float angle = 0.f;
    float angVel = 0.f;
    float mass = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    BodyType type = BodyType::Dynamic;

    [[nodiscard]] static constexpr BoxParams fromSize(Vec2 p, Vec2 sz) noexcept {
        return BoxParams{.pos = p, .half = sz * 0.5f};
    }
    [[nodiscard]] static constexpr BoxParams fromLowerUpper(Vec2 lo, Vec2 hi) noexcept {
        return BoxParams{.pos = (lo + hi) * 0.5f, .half = (hi - lo) * 0.5f};
    }
    [[nodiscard]] static constexpr BoxParams fromAABB(AABB box) noexcept {
        return BoxParams{.pos = box.center(), .half = box.half()};
    }
};

// Params de capsula: {.pos=..., .radius=8.f, .length=24.f}.
struct CapsuleParams {
    Vec2 pos{};
    float radius = 10.f;
    float length = 20.f;
    Vec2 dir{0.f, 1.f};
    Vec2 vel{};
    float angle = 0.f;
    float angVel = 0.f;
    float mass = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    BodyType type = BodyType::Dynamic;
};

// Params de poligono: {.pos=..., .verts={{...},{...}}}.
struct PolygonParams {
    Vec2 pos{};
    std::vector<Vec2> verts{};
    Vec2 vel{};
    float angle = 0.f;
    float angVel = 0.f;
    float mass = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    BodyType type = BodyType::Dynamic;
};

// Params de segmento (por defecto Static: suele ser colisor de suelo).
struct SegmentParams {
    Vec2 pos{};
    Vec2 a{};
    Vec2 b{100.f, 0.f};
    float radius = 0.f;
    Vec2 vel{};
    float angle = 0.f;
    float angVel = 0.f;
    float mass = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    BodyType type = BodyType::Static;
};

// Area de la forma local.
[[nodiscard]] inline float shapeArea(Shape const& shape) {
    return std::visit(
        [](auto const& s) -> float {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return s.area();
            } else if constexpr (std::is_same_v<T, AABB>) {
                return s.area();
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                return s.area();
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return s.area();
            } else {
                return s.radius > 0.f ? Capsule::fromSegment(s.a, s.b, s.radius).area() : 0.f;
            }
        },
        shape);
}
// Radio de la esfera que envuelve la forma alrededor del origen local.
[[nodiscard]] inline float boundingRadius(Shape const& shape) {
    return std::visit(
        [](auto const& s) -> float {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return s.radius;
            } else if constexpr (std::is_same_v<T, AABB>) {
                float m = s.lower.length();
                m = std::max(m, s.upper.length());
                m = std::max(m, Vec2{s.lower.x, s.upper.y}.length());
                m = std::max(m, Vec2{s.upper.x, s.lower.y}.length());
                return m;
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                float m = 0.f;
                for (auto v : s.verts) {
                    m = std::max(m, v.length());
                }
                return m;
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return s.capA().length() + s.radius;
            } else {
                return std::max(s.a.length(), s.b.length()) + s.radius;
            }
        },
        shape);
}
// Propiedades de masa por densidad; inv = 0 si type != Dynamic.
[[nodiscard]] inline MassProps massProperties(Shape const& shape, float density, BodyType type) {
    MassProps props;
    std::visit(
        [&props, density](auto const& s) {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                props.center = {};
                float m = density * s.area();
                props.mass = m > 0.f ? m : 0.f;
                props.inertia = 0.5f * props.mass * s.radius * s.radius;
            } else if constexpr (std::is_same_v<T, AABB>) {
                props.center = s.center();
                float w = s.width();
                float h = s.height();
                float m = density * w * h;
                props.mass = m > 0.f ? m : 0.f;
                props.inertia = props.mass * (w * w + h * h) / 12.f;
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                props = s.massProps(density);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                props.center = {};
                float mid = s.midLength();
                float rectMass = density * 2.f * s.radius * mid;
                float circleMass = density * kPi * s.radius * s.radius;
                float m = rectMass + circleMass;
                props.mass = m > 0.f ? m : 0.f;
                float rectInertia = rectMass * (mid * mid + 4.f * s.radius * s.radius) / 12.f;
                float circleInertia =
                    0.5f * circleMass * s.radius * s.radius + circleMass * (mid * mid) / 4.f;
                props.inertia = rectInertia + circleInertia;
            } else {
                Vec2 mid = (s.a + s.b) * 0.5f;
                props.center = mid;
                if (s.radius <= 0.f) {
                    props.mass = 0.f;
                    props.inertia = 0.f;
                } else {
                    Capsule cap = Capsule::fromSegment(s.a, s.b, s.radius);
                    float midLen = cap.midLength();
                    float rectMass = density * 2.f * cap.radius * midLen;
                    float circleMass = density * kPi * cap.radius * cap.radius;
                    float m = rectMass + circleMass;
                    props.mass = m > 0.f ? m : 0.f;
                    float rectInertia =
                        rectMass * (midLen * midLen + 4.f * cap.radius * cap.radius) / 12.f;
                    float circleInertia = 0.5f * circleMass * cap.radius * cap.radius +
                                          circleMass * (midLen * midLen) / 4.f;
                    props.inertia = rectInertia + circleInertia;
                }
            }
        },
        shape);
    if (type == BodyType::Dynamic && props.mass > 1e-8f) {
        props.invMass = 1.f / props.mass;
    } else {
        props.invMass = 0.f;
    }
    if (type == BodyType::Dynamic && props.inertia > 1e-8f) {
        props.invInertia = 1.f / props.inertia;
    } else {
        props.invInertia = 0.f;
    }
    return props;
}
// Punto local transformado a mundo: local.rotated(angle) + pos.
[[nodiscard]] inline Vec2 transformPoint(Vec2 local, Vec2 pos, float angle) noexcept {
    return local.rotated(angle) + pos;
}
// Alias de transformPoint.
[[nodiscard]] inline Vec2 localToWorld(Vec2 local, Vec2 pos, float angle) noexcept {
    return transformPoint(local, pos, angle);
}
// Punto mundo transformado a local: (world - pos).rotated(-angle).
[[nodiscard]] inline Vec2 worldToLocal(Vec2 world, Vec2 pos, float angle) noexcept {
    return (world - pos).rotated(-angle);
}
// Forma transformada a mundo (Circle/Capsule conservan tamano; AABB se envuelve).
[[nodiscard]] inline Shape transformShape(Shape const& shape, Vec2 pos, float angle) {
    return std::visit(
        [pos, angle](auto const& s) -> Shape {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return s;
            } else if constexpr (std::is_same_v<T, AABB>) {
                Vec2 corners[4] = {s.lower, s.upper, Vec2{s.lower.x, s.upper.y},
                                   Vec2{s.upper.x, s.lower.y}};
                Vec2 pts[4];
                for (int i = 0; i < 4; ++i) {
                    pts[i] = corners[i].rotated(angle) + pos;
                }
                Vec2 lo = pts[0];
                Vec2 hi = pts[0];
                for (int i = 1; i < 4; ++i) {
                    lo = min(lo, pts[i]);
                    hi = max(hi, pts[i]);
                }
                return AABB{lo, hi};
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                ConvexPolygon out;
                out.verts = s.transform(pos, angle);
                return out;
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return Capsule{s.radius, s.length, s.dir.rotated(angle)};
            } else {
                return Segment{s.a.rotated(angle) + pos, s.b.rotated(angle) + pos, s.radius};
            }
        },
        shape);
}
// AABB mundo de la forma dado su cuerpo (pos/angle).
[[nodiscard]] inline AABB shapeAABB(Shape const& shape, Vec2 bodyPos, float bodyAngle) {
    return std::visit(
        [bodyPos, bodyAngle](auto const& s) -> AABB {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                Vec2 r{s.radius, s.radius};
                return {bodyPos - r, bodyPos + r};
            } else if constexpr (std::is_same_v<T, AABB>) {
                Vec2 corners[4] = {s.lower, s.upper, Vec2{s.lower.x, s.upper.y},
                                   Vec2{s.upper.x, s.lower.y}};
                Vec2 pts[4];
                for (int i = 0; i < 4; ++i) {
                    pts[i] = corners[i].rotated(bodyAngle) + bodyPos;
                }
                Vec2 lo = pts[0];
                Vec2 hi = pts[0];
                for (int i = 1; i < 4; ++i) {
                    lo = min(lo, pts[i]);
                    hi = max(hi, pts[i]);
                }
                return AABB{lo, hi};
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                return s.aabb(bodyPos, bodyAngle);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return s.aabb(bodyPos, bodyAngle);
            } else {
                return s.aabb(bodyPos, bodyAngle);
            }
        },
        shape);
}
// Test de punto en espacio local.
[[nodiscard]] inline bool testPoint(Shape const& shape, Vec2 localPoint) {
    return std::visit(
        [localPoint](auto const& s) -> bool {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return s.testPoint(localPoint);
            } else if constexpr (std::is_same_v<T, AABB>) {
                return s.testPoint(localPoint);
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                return s.testPoint(localPoint);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return s.testPoint(localPoint);
            } else {
                return s.testPoint(localPoint);
            }
        },
        shape);
}
// Punto mundo dentro de la forma del cuerpo (pos/angle).
[[nodiscard]] inline bool pointInside(Shape const& shape, Vec2 worldPoint, Vec2 bodyPos,
                                      float bodyAngle) {
    return testPoint(shape, worldToLocal(worldPoint, bodyPos, bodyAngle));
}
// Punto mas cercano en espacio local (superficie; si esta dentro, el propio punto).
[[nodiscard]] inline Vec2 closestPoint(Shape const& shape, Vec2 localPoint) {
    return std::visit(
        [localPoint](auto const& s) -> Vec2 {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, Circle>) {
                return s.closestPoint(localPoint);
            } else if constexpr (std::is_same_v<T, AABB>) {
                return s.closestPoint(localPoint);
            } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                return s.closestPoint(localPoint);
            } else if constexpr (std::is_same_v<T, Capsule>) {
                return s.closestPoint(localPoint);
            } else {
                return s.closestSurfacePoint(localPoint);
            }
        },
        shape);
}

} // namespace phys2d
