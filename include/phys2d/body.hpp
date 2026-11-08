#pragma once
#include "phys2d/shapes.hpp"
#include "phys2d/vec2.hpp"
#include <cmath>

namespace phys2d {

using BodyId = int;

struct Body {
    Vec2 pos{};
    Vec2 vel{};
    Vec2 force{};
    float angle = 0.f;
    float angVel = 0.f;
    float torque = 0.f;
    float mass = 1.f;
    float invMass = 1.f;
    float inertia = 1.f;
    float invInertia = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    bool allowSleep = true;
    bool isSleeping = false;
    bool fixedRotation = false;
    bool bullet = false;
    float sleepTimer = 0.f;
    BodyType type = BodyType::Dynamic;
    Shape shape = Circle{};

    [[nodiscard]] constexpr bool isStatic() const noexcept {
        return type == BodyType::Static;
    }
    [[nodiscard]] constexpr bool isKinematic() const noexcept {
        return type == BodyType::Kinematic;
    }
    [[nodiscard]] constexpr bool isDynamic() const noexcept {
        return type == BodyType::Dynamic;
    }
    [[nodiscard]] constexpr bool isAwake() const noexcept {
        return !isSleeping;
    }

    void setStatic() noexcept {
        type = BodyType::Static;
        isSleeping = false;
        invMass = 0.f;
        invInertia = 0.f;
    }

    void setKinematic() noexcept {
        type = BodyType::Kinematic;
        isSleeping = false;
        invMass = 0.f;
        invInertia = 0.f;
    }

    void setDynamic(float m, float inertiaParam = 0.f) noexcept {
        type = BodyType::Dynamic;
        isSleeping = false;
        mass = m > 1e-8f ? m : 1e-8f;
        invMass = 1.f / mass;
        inertia = inertiaParam > 1e-8f ? inertiaParam : mass;
        invInertia = fixedRotation ? 0.f : 1.f / inertia;
    }

    void updateInertia() noexcept {
        if (type != BodyType::Dynamic) {
            invInertia = 0.f;
            return;
        }
        float area = shapeArea(shape);
        if (area > 1e-8f && mass > 1e-8f) {
            float density = mass / area;
            MassProps props = massProperties(shape, density, BodyType::Dynamic);
            float computed = props.inertia + mass * props.center.length2();
            if (computed > 1e-8f) {
                inertia = computed;
            }
        }
        if (inertia <= 1e-8f) {
            inertia = mass > 1e-8f ? mass : 1.f;
        }
        invInertia = fixedRotation ? 0.f : 1.f / inertia;
    }

    void wake() noexcept {
        isSleeping = false;
        sleepTimer = 0.f;
        if (type == BodyType::Dynamic) {
            invMass = mass > 1e-8f ? 1.f / mass : 0.f;
            invInertia = fixedRotation || inertia <= 1e-8f ? 0.f : 1.f / inertia;
        }
    }

    void sleep() noexcept {
        isSleeping = true;
        vel = {};
        angVel = 0.f;
        sleepTimer = 0.f;
        invMass = 0.f;
        invInertia = 0.f;
    }

    void applyForce(Vec2 f) noexcept {
        if (isSleeping) {
            wake();
        }
        force += f;
    }

    void applyForce(Vec2 f, Vec2 worldPoint) noexcept {
        applyForce(f);
        Vec2 r = worldPoint - pos;
        torque += r.cross(f);
    }

    void applyLinearImpulse(Vec2 imp) noexcept {
        if (isSleeping) {
            wake();
        }
        if (invMass <= 0.f) {
            return;
        }
        vel += imp * invMass;
    }

    void applyLinearImpulse(Vec2 imp, Vec2 worldPoint) noexcept {
        if (isSleeping) {
            wake();
        }
        if (invMass <= 0.f) {
            return;
        }
        vel += imp * invMass;
        if (invInertia > 0.f) {
            Vec2 r = worldPoint - pos;
            angVel += invInertia * r.cross(imp);
        }
    }

    void applyAngularImpulse(float imp) noexcept {
        if (isSleeping) {
            wake();
        }
        if (invInertia <= 0.f) {
            return;
        }
        angVel += imp * invInertia;
    }

    void clearForces() noexcept {
        force = {};
        torque = 0.f;
    }

    [[nodiscard]] float boundingRadius() const noexcept {
        return ::phys2d::boundingRadius(shape);
    }

    [[nodiscard]] Vec2 worldPoint(Vec2 local) const noexcept {
        return local.rotated(angle) + pos;
    }
    [[nodiscard]] Vec2 localPoint(Vec2 world) const noexcept {
        return (world - pos).rotated(-angle);
    }

    void configure(BodyParams const& p) {
        applyCommon(p.pos, p.vel, p.angle, p.angVel, p.mass, p.friction, p.restitution,
                    p.linearDamping, p.angularDamping, p.type);
        shape = Circle{10.f};
        updateInertia();
    }

    void configure(CircleParams const& p) {
        applyCommon(p.pos, p.vel, p.angle, p.angVel, p.mass, p.friction, p.restitution,
                    p.linearDamping, p.angularDamping, p.type);
        shape = Circle{p.radius};
        updateInertia();
    }

    void configure(BoxParams const& p) {
        applyCommon(p.pos, p.vel, p.angle, p.angVel, p.mass, p.friction, p.restitution,
                    p.linearDamping, p.angularDamping, p.type);
        shape = AABB::fromCenterHalf(Vec2{}, p.half);
        updateInertia();
    }

    void configure(CapsuleParams const& p) {
        applyCommon(p.pos, p.vel, p.angle, p.angVel, p.mass, p.friction, p.restitution,
                    p.linearDamping, p.angularDamping, p.type);
        // dir no nulo y finito se normaliza aquí; dir nulo/NaN lo rechaza validateBody.
        Vec2 const dir = p.dir.length2() > 1e-12f ? p.dir.normalized() : p.dir;
        shape = Capsule{p.radius, p.length, dir};
        updateInertia();
    }

    void configure(PolygonParams const& p) {
        applyCommon(p.pos, p.vel, p.angle, p.angVel, p.mass, p.friction, p.restitution,
                    p.linearDamping, p.angularDamping, p.type);
        shape = ConvexPolygon{p.verts};
        updateInertia();
    }

    void configure(SegmentParams const& p) {
        applyCommon(p.pos, p.vel, p.angle, p.angVel, p.mass, p.friction, p.restitution,
                    p.linearDamping, p.angularDamping, p.type);
        shape = Segment{p.a, p.b, p.radius};
        updateInertia();
    }

private:
    void applyCommon(Vec2 p, Vec2 v, float a, float av, float m, float fric, float rest,
                     float linDamp, float angDamp, BodyType bodyType) noexcept {
        pos = p;
        vel = v;
        angle = a;
        angVel = av;
        friction = fric;
        restitution = rest;
        linearDamping = linDamp;
        angularDamping = angDamp;
        force = {};
        torque = 0.f;
        sleepTimer = 0.f;
        if (bodyType == BodyType::Static) {
            setStatic();
        } else if (bodyType == BodyType::Kinematic) {
            setKinematic();
        } else {
            setDynamic(m);
        }
    }
};

} // namespace phys2d
