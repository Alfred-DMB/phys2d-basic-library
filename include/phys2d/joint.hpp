#pragma once
#include "phys2d/body.hpp"
#include "phys2d/shapes.hpp"
#include "phys2d/vec2.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace phys2d {

enum class JointType { Distance, Revolute, Prismatic, Pulley, Spring, Weld, Gear, Rope, Slider };

[[nodiscard]] inline Vec2 pointVelocity(const Body& body, Vec2 arm) noexcept {
    return body.vel + arm.perpendicular() * body.angVel;
}

inline void applyImpulse(Body& body, Vec2 impulse, Vec2 arm) noexcept {
    body.vel += impulse * body.invMass;
    body.angVel += body.invInertia * arm.cross(impulse);
}

inline void applyAngularImpulse(Body& body, float impulse) noexcept {
    body.angVel += body.invInertia * impulse;
}

struct Joint {
    int a = -1;
    int b = -1;
    JointType type = JointType::Distance;
    float maxImpulse = 1.0e30f;
    float beta = 0.2f;
    float gamma = 0.f;
    Vec2 bias{};
    float biasScalar = 0.f;
    float biasAngle = 0.f;
    float mass = 0.f;
    float massAngular = 0.f;
    Vec2 impulse{};
    float impulseScalar = 0.f;
    float angularImpulse = 0.f;
    float motorImpulse = 0.f;
    Vec2 ra{};
    Vec2 rb{};
    Vec2 unit{};

    virtual ~Joint() = default;
    [[nodiscard]] virtual std::unique_ptr<Joint> clone() const = 0;
    virtual void prepare(const Body& bodyA, const Body& bodyB, float invDt) = 0;
    virtual void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) = 0;

    inline void solve(Body& bodyA, Body& bodyB) {
        solve(static_cast<const Body&>(bodyA), bodyA, static_cast<const Body&>(bodyB), bodyB);
    }

    [[nodiscard]] constexpr bool isType(JointType kind) const noexcept {
        return type == kind;
    }
};

struct Distance : Joint {
    float restLength = 0.f;
    float stiffness = 1.f;
    float damping = 0.f;
    Vec2 anchorA{};
    Vec2 anchorB{};
    float distCached = 0.f;

    Distance() = default;
    Distance(int idA, int idB, float restLen, Vec2 anchorALocal, Vec2 anchorBLocal) noexcept
        : restLength(restLen), anchorA(anchorALocal), anchorB(anchorBLocal) {
        a = idA;
        b = idB;
        type = JointType::Distance;
    }
    Distance(int idA, int idB, float restLen) noexcept : restLength(restLen) {
        a = idA;
        b = idB;
        type = JointType::Distance;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Distance>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        Vec2 delta = worldB - worldA;
        float len = delta.length();
        distCached = len;
        unit = len > 1e-8f ? delta / len : Vec2{1.f, 0.f};
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        float invMassA = bodyA.invMass;
        float invMassB = bodyB.invMass;
        float cA = ra.cross(unit);
        float cB = rb.cross(unit);
        float effK = invMassA + invMassB + bodyA.invInertia * cA * cA + bodyB.invInertia * cB * cB;
        mass = effK > 1e-8f ? 1.f / effK : 0.f;
        float gapC = len - restLength;
        biasScalar = beta * invDt * gapC;
        bias = unit * biasScalar;
        impulseScalar = 0.f;
        impulse = {};
        if (damping > 0.f && len > 1e-8f) {
            Vec2 relVel = pointVelocity(bodyB, rb) - pointVelocity(bodyA, ra);
            float const dt = invDt > 1e-8f ? 1.f / invDt : 1.f / 60.f;
            biasScalar += damping * relVel.dot(unit) * dt * effK;
            bias = unit * biasScalar;
        }
        if (stiffness < 1.f) {
            mass *= stiffness;
        }
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (mass <= 0.f) {
            return;
        }
        Vec2 relVel = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
        float velAlong = relVel.dot(unit);
        float velConstraint = velAlong + biasScalar + gamma * impulseScalar;
        float lambda = -mass * velConstraint;
        float oldImpulse = impulseScalar;
        impulseScalar = std::clamp(impulseScalar + lambda, -maxImpulse, maxImpulse);
        float deltaImpulse = impulseScalar - oldImpulse;
        Vec2 applied = unit * deltaImpulse;
        applyImpulse(bodyAmut, -applied, ra);
        applyImpulse(bodyBmut, applied, rb);
        (void)bodyA;
        (void)bodyB;
        impulse = unit * impulseScalar;
    }
};

struct Revolute : Joint {
    Vec2 anchorA{};
    Vec2 anchorB{};
    float maxMotorTorque = 1.0e30f;
    bool enableMotor = false;
    float motorSpeed = 0.f;
    float k11 = 0.f;
    float k12 = 0.f;
    float k22 = 0.f;
    float invDet = 0.f;
    Vec2 kInvC0{};
    Vec2 kInvC1{};

    Revolute() = default;
    Revolute(int idA, int idB, Vec2 anchorALocal, Vec2 anchorBLocal) noexcept
        : anchorA(anchorALocal), anchorB(anchorBLocal) {
        a = idA;
        b = idB;
        type = JointType::Revolute;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Revolute>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        Vec2 gapVec = worldB - worldA;
        bias = gapVec * (beta * invDt);
        k11 = bodyA.invMass + bodyB.invMass + bodyA.invInertia * ra.y * ra.y +
              bodyB.invInertia * rb.y * rb.y;
        k22 = bodyA.invMass + bodyB.invMass + bodyA.invInertia * ra.x * ra.x +
              bodyB.invInertia * rb.x * rb.x;
        k12 = -bodyA.invInertia * ra.x * ra.y - bodyB.invInertia * rb.x * rb.y;
        float detVal = k11 * k22 - k12 * k12;
        invDet = detVal > 1e-12f ? 1.f / detVal : 0.f;
        kInvC0 = Vec2{k22 * invDet, -k12 * invDet};
        kInvC1 = Vec2{-k12 * invDet, k11 * invDet};
        massAngular = (bodyA.invInertia + bodyB.invInertia) > 1e-8f
                          ? 1.f / (bodyA.invInertia + bodyB.invInertia)
                          : 0.f;
        impulse = {};
        angularImpulse = 0.f;
        motorImpulse = 0.f;
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (invDet > 0.f) {
            Vec2 relVel = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
            Vec2 rhs = relVel + bias + impulse * gamma;
            Vec2 step{-rhs.dot(kInvC0), -rhs.dot(kInvC1)};
            Vec2 newImpulse = impulse + step;
            float len = newImpulse.length();
            float clamped = len > maxImpulse ? maxImpulse / len : 1.f;
            newImpulse = newImpulse * clamped;
            Vec2 deltaImpulse = newImpulse - impulse;
            applyImpulse(bodyAmut, -deltaImpulse, ra);
            applyImpulse(bodyBmut, deltaImpulse, rb);
            impulse = newImpulse;
        }
        if (enableMotor && massAngular > 0.f) {
            float relAng = bodyBmut.angVel - bodyAmut.angVel;
            float desired = motorSpeed;
            float constraint = relAng - desired + gamma * motorImpulse;
            float lambda = -massAngular * constraint;
            float oldMotor = motorImpulse;
            motorImpulse = std::clamp(motorImpulse + lambda, -maxMotorTorque, maxMotorTorque);
            float applied = motorImpulse - oldMotor;
            applyAngularImpulse(bodyAmut, -applied);
            applyAngularImpulse(bodyBmut, applied);
        }
        (void)bodyA;
        (void)bodyB;
    }
};

struct Prismatic : Joint {
    Vec2 axis{1.f, 0.f};
    float lower = -1.0e30f;
    float upper = 1.0e30f;
    Vec2 anchorA{};
    Vec2 anchorB{};
    bool enableLimit = true;
    Vec2 perp{};
    Vec2 axisNorm{1.f, 0.f};
    float massPerp = 0.f;
    float massAxis = 0.f;
    float biasPerp = 0.f;
    float biasAxis = 0.f;
    float dispCached = 0.f;
    float limitImpulse = 0.f;

    Prismatic() = default;
    Prismatic(int idA, int idB, Vec2 ax, float low, float high) noexcept
        : axis(ax), lower(low), upper(high) {
        a = idA;
        b = idB;
        type = JointType::Prismatic;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Prismatic>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        float axLen = axis.length();
        Vec2 axNorm = axLen > 1e-8f ? axis / axLen : Vec2{1.f, 0.f};
        axisNorm = axNorm;
        perp = axNorm.perpendicular();
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        Vec2 gapVec = worldB - worldA;
        float gapPerp = gapVec.dot(perp);
        dispCached = gapVec.dot(axNorm);
        biasPerp = beta * invDt * gapPerp;
        biasAxis = 0.f;
        bool outsideLow = dispCached < lower;
        bool outsideHigh = dispCached > upper;
        if (enableLimit && (outsideLow || outsideHigh)) {
            float target = std::clamp(dispCached, lower, upper);
            biasAxis = beta * invDt * (dispCached - target);
        }
        bias = perp * biasPerp + axNorm * biasAxis;
        float cPerpA = ra.cross(perp);
        float cPerpB = rb.cross(perp);
        float kPerp = bodyA.invMass + bodyB.invMass + bodyA.invInertia * cPerpA * cPerpA +
                      bodyB.invInertia * cPerpB * cPerpB;
        massPerp = kPerp > 1e-8f ? 1.f / kPerp : 0.f;
        float cAxA = ra.cross(axNorm);
        float cAxB = rb.cross(axNorm);
        float kAx = bodyA.invMass + bodyB.invMass + bodyA.invInertia * cAxA * cAxA +
                    bodyB.invInertia * cAxB * cAxB;
        massAxis = kAx > 1e-8f ? 1.f / kAx : 0.f;
        mass = massPerp;
        impulseScalar = 0.f;
        limitImpulse = 0.f;
        impulse = {};
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (massPerp > 0.f) {
            Vec2 relVel = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
            float perpVel = relVel.dot(perp);
            float constraint = perpVel + biasPerp + gamma * impulseScalar;
            float lambda = -massPerp * constraint;
            float oldImpulse = impulseScalar;
            impulseScalar = std::clamp(impulseScalar + lambda, -maxImpulse, maxImpulse);
            float deltaImpulse = impulseScalar - oldImpulse;
            Vec2 applied = perp * deltaImpulse;
            applyImpulse(bodyAmut, -applied, ra);
            applyImpulse(bodyBmut, applied, rb);
        }
        if (enableLimit && massAxis > 0.f) {
            bool outsideLow = dispCached < lower;
            bool outsideHigh = dispCached > upper;
            if (outsideLow || outsideHigh) {
                Vec2 relVel2 = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
                float axVel = relVel2.dot(axisNorm);
                float constraintAx = axVel + biasAxis;
                float lambdaAx = -massAxis * constraintAx;
                float oldLim = limitImpulse;
                float newLim = limitImpulse + lambdaAx;
                if (outsideLow) {
                    newLim = std::max(newLim, 0.f);
                }
                if (outsideHigh) {
                    newLim = std::min(newLim, 0.f);
                }
                newLim = std::clamp(newLim, -maxImpulse, maxImpulse);
                float deltaLim = newLim - oldLim;
                Vec2 appliedLim = axisNorm * deltaLim;
                applyImpulse(bodyAmut, -appliedLim, ra);
                applyImpulse(bodyBmut, appliedLim, rb);
                limitImpulse = newLim;
            }
        }
        (void)bodyA;
        (void)bodyB;
    }
};

struct Slider : Joint {
    Vec2 axis{1.f, 0.f};
    float lower = -1.0e30f;
    float upper = 1.0e30f;
    Vec2 anchorA{};
    Vec2 anchorB{};
    Prismatic inner;

    Slider() = default;
    Slider(int idA, int idB, Vec2 ax, float low, float high) noexcept
        : axis(ax), lower(low), upper(high) {
        a = idA;
        b = idB;
        type = JointType::Slider;
        inner.a = idA;
        inner.b = idB;
        inner.type = JointType::Prismatic;
        inner.axis = ax;
        inner.lower = low;
        inner.upper = high;
        inner.maxImpulse = maxImpulse;
        inner.beta = beta;
        inner.gamma = gamma;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Slider>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        inner.anchorA = anchorA;
        inner.anchorB = anchorB;
        inner.maxImpulse = maxImpulse;
        inner.beta = beta;
        inner.gamma = gamma;
        inner.prepare(bodyA, bodyB, invDt);
        ra = inner.ra;
        rb = inner.rb;
        bias = inner.bias;
        biasScalar = inner.biasScalar;
        mass = inner.mass;
        massAngular = inner.massAngular;
        biasAngle = inner.biasAngle;
        impulse = inner.impulse;
        impulseScalar = inner.impulseScalar;
        angularImpulse = inner.angularImpulse;
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        inner.solve(bodyA, bodyAmut, bodyB, bodyBmut);
        impulse = inner.impulse;
        impulseScalar = inner.impulseScalar;
    }
};

struct Pulley : Joint {
    Vec2 anchorA{};
    Vec2 anchorB{};
    Vec2 groundA{};
    Vec2 groundB{};
    float ratio = 1.f;
    float totalLength = 0.f;
    float restTotal = 0.f;
    bool restTotalSet = false;
    Vec2 unitA{};
    Vec2 unitB{};

    Pulley() = default;
    Pulley(int idA, int idB, Vec2 anchorALocal, Vec2 anchorBLocal, Vec2 groundAWorld,
           Vec2 groundBWorld, float ratioVal) noexcept
        : anchorA(anchorALocal), anchorB(anchorBLocal), groundA(groundAWorld),
          groundB(groundBWorld), ratio(ratioVal) {
        a = idA;
        b = idB;
        type = JointType::Pulley;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Pulley>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        Vec2 segA = worldA - groundA;
        Vec2 segB = worldB - groundB;
        float lenA = segA.length();
        float lenB = segB.length();
        unitA = lenA > 1e-8f ? segA / lenA : Vec2{1.f, 0.f};
        unitB = lenB > 1e-8f ? segB / lenB : Vec2{1.f, 0.f};
        totalLength = lenA + ratio * lenB;
        if (!restTotalSet) {
            restTotal = totalLength;
            restTotalSet = true;
        }
        float gapC = totalLength - restTotal;
        biasScalar = beta * invDt * gapC;
        bias = unitA * biasScalar;
        float cA = ra.cross(unitA);
        float cB = rb.cross(unitB);
        float effK = bodyA.invMass + bodyB.invMass * ratio * ratio + bodyA.invInertia * cA * cA +
                     bodyB.invInertia * cB * cB * ratio * ratio;
        mass = effK > 1e-8f ? 1.f / effK : 0.f;
        impulseScalar = 0.f;
        impulse = {};
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (mass <= 0.f) {
            return;
        }
        Vec2 velA = pointVelocity(bodyAmut, ra);
        Vec2 velB = pointVelocity(bodyBmut, rb);
        float velAlong = velA.dot(unitA) + ratio * velB.dot(unitB);
        float constraint = velAlong + biasScalar + gamma * impulseScalar;
        float lambda = -mass * constraint;
        float oldImpulse = impulseScalar;
        impulseScalar = std::clamp(impulseScalar + lambda, -maxImpulse, maxImpulse);
        float deltaImpulse = impulseScalar - oldImpulse;
        Vec2 appliedA = unitA * deltaImpulse;
        Vec2 appliedB = unitB * (deltaImpulse * ratio);
        // El gradiente del constraint es (unitA, ratio*unitB): el impulso
        // generalizado se aplica con ese mismo signo en ambos cuerpos.
        applyImpulse(bodyAmut, appliedA, ra);
        applyImpulse(bodyBmut, appliedB, rb);
        (void)bodyA;
        (void)bodyB;
    }
};

struct Spring : Joint {
    float stiffness = 100.f;
    float damping = 0.f;
    float restLength = 0.f;
    Vec2 anchorA{};
    Vec2 anchorB{};
    float distCached = 0.f;

    Spring() = default;
    Spring(int idA, int idB, float restLen, float stiff, float damp) noexcept
        : stiffness(stiff), damping(damp), restLength(restLen) {
        a = idA;
        b = idB;
        type = JointType::Spring;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Spring>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        Vec2 delta = worldB - worldA;
        float len = delta.length();
        distCached = len;
        unit = len > 1e-8f ? delta / len : Vec2{1.f, 0.f};
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        float cA = ra.cross(unit);
        float cB = rb.cross(unit);
        float effK =
            bodyA.invMass + bodyB.invMass + bodyA.invInertia * cA * cA + bodyB.invInertia * cB * cB;
        float dt = invDt > 1e-8f ? 1.f / invDt : 1.f / 60.f;
        // Muelle explícito: la fuerza F = k*C + c*v se integra en dt como
        // bias (unidad m/s) para que la restricción en solve() la aplique
        // sin regularizar en exceso. El término gamma no se usa.
        mass = effK > 1e-8f ? 1.f / effK : 0.f;
        gamma = 0.f;
        Vec2 relVel = pointVelocity(bodyB, rb) - pointVelocity(bodyA, ra);
        float relVelAlong = relVel.dot(unit);
        float gapC = len - restLength;
        float force = stiffness * gapC + damping * relVelAlong;
        biasScalar = force * dt * effK;
        bias = unit * biasScalar;
        impulseScalar = 0.f;
        impulse = {};
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (mass <= 0.f) {
            return;
        }
        Vec2 relVel = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
        float velAlong = relVel.dot(unit);
        float velConstraint = velAlong + biasScalar + gamma * impulseScalar;
        float lambda = -mass * velConstraint;
        float oldImpulse = impulseScalar;
        impulseScalar = std::clamp(impulseScalar + lambda, -maxImpulse, maxImpulse);
        float deltaImpulse = impulseScalar - oldImpulse;
        Vec2 applied = unit * deltaImpulse;
        applyImpulse(bodyAmut, -applied, ra);
        applyImpulse(bodyBmut, applied, rb);
        (void)bodyA;
        (void)bodyB;
    }
};

struct Weld : Joint {
    Vec2 anchorA{};
    Vec2 anchorB{};
    float referenceAngle = 0.f;
    float k11 = 0.f;
    float k12 = 0.f;
    float k22 = 0.f;
    float invDet = 0.f;
    Vec2 kInvC0{};
    Vec2 kInvC1{};

    Weld() = default;
    Weld(int idA, int idB, Vec2 anchorALocal, Vec2 anchorBLocal) noexcept
        : anchorA(anchorALocal), anchorB(anchorBLocal) {
        a = idA;
        b = idB;
        type = JointType::Weld;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Weld>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        Vec2 gapVec = worldB - worldA;
        bias = gapVec * (beta * invDt);
        k11 = bodyA.invMass + bodyB.invMass + bodyA.invInertia * ra.y * ra.y +
              bodyB.invInertia * rb.y * rb.y;
        k22 = bodyA.invMass + bodyB.invMass + bodyA.invInertia * ra.x * ra.x +
              bodyB.invInertia * rb.x * rb.x;
        k12 = -bodyA.invInertia * ra.x * ra.y - bodyB.invInertia * rb.x * rb.y;
        float detVal = k11 * k22 - k12 * k12;
        invDet = detVal > 1e-12f ? 1.f / detVal : 0.f;
        kInvC0 = Vec2{k22 * invDet, -k12 * invDet};
        kInvC1 = Vec2{-k12 * invDet, k11 * invDet};
        float sumInvI = bodyA.invInertia + bodyB.invInertia;
        massAngular = sumInvI > 1e-8f ? 1.f / sumInvI : 0.f;
        float angleDiff = bodyB.angle - bodyA.angle - referenceAngle;
        biasAngle = beta * invDt * angleDiff;
        impulse = {};
        angularImpulse = 0.f;
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (invDet > 0.f) {
            Vec2 relVel = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
            Vec2 rhs = relVel + bias + impulse * gamma;
            Vec2 step{-rhs.dot(kInvC0), -rhs.dot(kInvC1)};
            Vec2 newImpulse = impulse + step;
            float len = newImpulse.length();
            float clamped = len > maxImpulse ? maxImpulse / len : 1.f;
            newImpulse = newImpulse * clamped;
            Vec2 deltaImpulse = newImpulse - impulse;
            applyImpulse(bodyAmut, -deltaImpulse, ra);
            applyImpulse(bodyBmut, deltaImpulse, rb);
            impulse = newImpulse;
        }
        if (massAngular > 0.f) {
            float relAng = bodyBmut.angVel - bodyAmut.angVel;
            float constraintA = relAng + biasAngle + gamma * angularImpulse;
            float lambdaA = -massAngular * constraintA;
            float oldAng = angularImpulse;
            angularImpulse = std::clamp(angularImpulse + lambdaA, -maxImpulse, maxImpulse);
            float appliedAng = angularImpulse - oldAng;
            applyAngularImpulse(bodyAmut, -appliedAng);
            applyAngularImpulse(bodyBmut, appliedAng);
        }
        (void)bodyA;
        (void)bodyB;
    }
};

struct Gear : Joint {
    float ratio = 1.f;
    float phase = 0.f;

    Gear() = default;
    Gear(int idA, int idB, float ratioVal) noexcept : ratio(ratioVal) {
        a = idA;
        b = idB;
        type = JointType::Gear;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Gear>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        float effK = bodyB.invInertia + ratio * ratio * bodyA.invInertia;
        massAngular = effK > 1e-8f ? 1.f / effK : 0.f;
        float angleErr = bodyB.angle + ratio * bodyA.angle - phase;
        biasAngle = beta * invDt * angleErr;
        angularImpulse = 0.f;
        (void)bodyA;
        (void)bodyB;
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (massAngular <= 0.f) {
            return;
        }
        float relAng = bodyBmut.angVel + ratio * bodyAmut.angVel;
        float constraint = relAng + biasAngle + gamma * angularImpulse;
        float lambda = -massAngular * constraint;
        float oldImpulse = angularImpulse;
        angularImpulse = std::clamp(angularImpulse + lambda, -maxImpulse, maxImpulse);
        float applied = angularImpulse - oldImpulse;
        // Constraint angB + ratio*angA: gradiente (ratio en A, 1 en B).
        applyAngularImpulse(bodyAmut, ratio * applied);
        applyAngularImpulse(bodyBmut, applied);
        (void)bodyA;
        (void)bodyB;
    }
};

struct Rope : Joint {
    float maxLength = 10.f;
    Vec2 anchorA{};
    Vec2 anchorB{};
    bool active = false;
    float distCached = 0.f;

    Rope() = default;
    Rope(int idA, int idB, float maxLen) noexcept : maxLength(maxLen) {
        a = idA;
        b = idB;
        type = JointType::Rope;
    }

    [[nodiscard]] std::unique_ptr<Joint> clone() const override {
        return std::make_unique<Rope>(*this);
    }

    void prepare(const Body& bodyA, const Body& bodyB, float invDt) override {
        Vec2 worldA = bodyA.worldPoint(anchorA);
        Vec2 worldB = bodyB.worldPoint(anchorB);
        Vec2 delta = worldB - worldA;
        float len = delta.length();
        distCached = len;
        unit = len > 1e-8f ? delta / len : Vec2{1.f, 0.f};
        ra = worldA - bodyA.pos;
        rb = worldB - bodyB.pos;
        active = len > maxLength;
        if (!active) {
            mass = 0.f;
            biasScalar = 0.f;
            bias = {};
            impulseScalar = 0.f;
            return;
        }
        float cA = ra.cross(unit);
        float cB = rb.cross(unit);
        float effK =
            bodyA.invMass + bodyB.invMass + bodyA.invInertia * cA * cA + bodyB.invInertia * cB * cB;
        mass = effK > 1e-8f ? 1.f / effK : 0.f;
        float gapC = len - maxLength;
        biasScalar = beta * invDt * gapC;
        bias = unit * biasScalar;
        impulseScalar = 0.f;
        impulse = {};
    }

    void solve(const Body& bodyA, Body& bodyAmut, const Body& bodyB, Body& bodyBmut) override {
        if (!active || mass <= 0.f) {
            return;
        }
        Vec2 relVel = pointVelocity(bodyBmut, rb) - pointVelocity(bodyAmut, ra);
        float velAlong = relVel.dot(unit);
        float velConstraint = velAlong + biasScalar + gamma * impulseScalar;
        float lambda = -mass * velConstraint;
        float oldImpulse = impulseScalar;
        float newImpulse = impulseScalar + lambda;
        // La cuerda solo tira: el impulso acumulado nunca empuja (<= 0).
        newImpulse = std::clamp(newImpulse, -maxImpulse, 0.f);
        impulseScalar = newImpulse;
        float deltaImpulse = impulseScalar - oldImpulse;
        Vec2 applied = unit * deltaImpulse;
        applyImpulse(bodyAmut, -applied, ra);
        applyImpulse(bodyBmut, applied, rb);
        (void)bodyA;
        (void)bodyB;
    }
};

} // namespace phys2d
