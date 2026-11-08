#pragma once
#include "phys2d/body.hpp"
#include "phys2d/collision.hpp"
#include "phys2d/joint.hpp"
#include "phys2d/vec2.hpp"

namespace phys2d {

// Free solvers delegate to joint.hpp classes (single source of truth for the math).

inline void solveDistance(Body& bodyA, Body& bodyB, float restLength, float dt,
                          float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Distance joint{0, 1, restLength};
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveDistance(Body& bodyA, Body& bodyB, const Manifold& manifold, float dt) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    // Las anclas son LOCALES: hay que convertir los puntos de contacto del
    // mundo una sola vez (worldPoint(localPoint(p)) == p). El reposo es la
    // longitud actual entre ambos puntos menos la penetración.
    Vec2 anchorLocalA = bodyA.localPoint(manifold.point);
    Vec2 anchorLocalB = bodyB.localPoint(manifold.point2);
    float rest = (manifold.point2 - manifold.point).length() - manifold.depth;
    rest = rest > 0.f ? rest : 0.f;
    Distance joint{0, 1, rest, anchorLocalA, anchorLocalB};
    joint.beta = 0.2f;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveRevolute(Body& bodyA, Body& bodyB, Vec2 anchorLocalA, Vec2 anchorLocalB, float dt,
                          float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Revolute joint{0, 1, anchorLocalA, anchorLocalB};
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveRevolute(Body& bodyA, Body& bodyB, const Manifold& manifold, float dt) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Vec2 anchorLocalA = bodyA.localPoint(manifold.point);
    Vec2 anchorLocalB = bodyB.localPoint(manifold.point2);
    Revolute joint{0, 1, anchorLocalA, anchorLocalB};
    joint.beta = 0.2f;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveSpring(Body& bodyA, Body& bodyB, float restLength, float stiffness, float damping,
                        float dt) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Spring joint{0, 1, restLength, stiffness, damping};
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveSpring(Body& bodyA, Body& bodyB, const Manifold& manifold, float dt,
                        float stiffness, float damping, float restLength) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Spring joint{0, 1, restLength, stiffness, damping};
    joint.anchorA = bodyA.localPoint(manifold.point);
    joint.anchorB = bodyB.localPoint(manifold.point2);
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solvePrismatic(Body& bodyA, Body& bodyB, Vec2 axis, float lower, float upper,
                           Vec2 anchorA, Vec2 anchorB, float dt, float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Prismatic joint{0, 1, axis, lower, upper};
    joint.anchorA = anchorA;
    joint.anchorB = anchorB;
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveWeld(Body& bodyA, Body& bodyB, Vec2 anchorA, Vec2 anchorB, float dt,
                      float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Weld joint{0, 1, anchorA, anchorB};
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveRope(Body& bodyA, Body& bodyB, float maxLength, float dt,
                      float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Rope joint{0, 1, maxLength};
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solveGear(Body& bodyA, Body& bodyB, float ratio, float dt, float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Gear joint{0, 1, ratio};
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

inline void solvePulley(Body& bodyA, Body& bodyB, Vec2 anchorA, Vec2 anchorB, Vec2 groundA,
                        Vec2 groundB, float ratio, float restTotal, float dt,
                        float beta = 0.2f) noexcept {
    float invDt = dt > 1e-8f ? 1.f / dt : 60.f;
    Pulley joint{0, 1, anchorA, anchorB, groundA, groundB, ratio};
    joint.groundA = groundA;
    joint.groundB = groundB;
    joint.restTotal = restTotal;
    joint.restTotalSet = true; // si no se marca, prepare() congela la longitud actual
    joint.beta = beta;
    joint.prepare(bodyA, bodyB, invDt);
    joint.solve(bodyA, bodyA, bodyB, bodyB);
}

} // namespace phys2d
