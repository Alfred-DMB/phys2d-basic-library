// Suite mínima sin dependencias: cada check falla con exit != 0.
#include "phys2d/phys2d.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <print>
#include <stdexcept>
#include <vector>

namespace {

int failures = 0;

void check(bool condition, char const* message) {
    if (!condition) {
        std::println("FAIL: {}", message);
        ++failures;
    }
}

bool near(float a, float b, float eps) {
    return std::abs(a - b) <= eps;
}

phys2d::World floorWorld() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    (void)world.addStaticBox(
        phys2d::BoxParams{.pos = {400.f, 590.f}, .half = {500.f, 10.f}, .friction = 0.8f});
    return world;
}

void testCircleRestsOnFloor() {
    phys2d::World world = floorWorld();
    int const c = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {100.f, 50.f}, .radius = 20.f, .friction = 0.8f});
    for (int i = 0; i < 360; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(c);
    check(near(body.pos.y, 560.f, 2.f), "círculo reposa sobre el suelo (y≈560)");
    check(near(body.vel.length(), 0.f, 1.f), "círculo en reposo tras caer");
}

void testRotatedBoxFlattens() {
    phys2d::World world = floorWorld();
    int const b = world.addDynamicBox(phys2d::BoxParams{
        .pos = {400.f, 400.f}, .half = {40.f, 10.f}, .angle = 0.5f, .friction = 0.8f});
    for (int i = 0; i < 360; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    check(std::abs(body.angle) < 0.05f, "caja rotada se aplana (torque del solver)");
    check(near(body.pos.y, 570.f, 3.f), "caja rotada reposa sobre el suelo");
}

void testStackOfFourBoxes() {
    phys2d::World world = floorWorld();
    std::array<int, 4> ids{};
    for (int i = 0; i < 4; ++i) {
        phys2d::BoxParams params{};
        params.pos = {400.f, 560.f - static_cast<float>(i) * 40.f - 200.f};
        params.half = {20.f, 20.f};
        params.friction = 0.8f;
        ids[static_cast<std::size_t>(i)] = world.addDynamicBox(params);
    }
    for (int i = 0; i < 360; ++i) {
        world.step(1.f / 60.f);
    }
    for (int i = 0; i < 4; ++i) {
        phys2d::Body const& body = world.body(ids[static_cast<std::size_t>(i)]);
        float const expectY = 560.f - static_cast<float>(i) * 40.f;
        check(near(body.pos.y, expectY, 2.f), "pila de 4 cajas sin jitter >2px en Y");
        check(std::abs(body.angle) < 0.05f, "pila de 4 cajas sin rotación");
    }
}

void testSettledStackSleeps() {
    phys2d::World world = floorWorld();
    std::array<int, 2> ids{};
    for (int i = 0; i < 2; ++i) {
        phys2d::BoxParams params{};
        params.pos = {400.f, 560.f - static_cast<float>(i) * 40.f - 120.f};
        params.half = {20.f, 20.f};
        params.friction = 0.8f;
        ids[static_cast<std::size_t>(i)] = world.addDynamicBox(params);
    }
    for (int i = 0; i < 600; ++i) {
        world.step(1.f / 60.f);
    }
    int sleeping = 0;
    for (int id : ids) {
        sleeping += world.body(id).isSleeping ? 1 : 0;
    }
    check(sleeping == 2, "pila asentada duerme (sleep activado por defecto)");
}

void testWorldRaycast() {
    phys2d::World world = floorWorld();
    (void)world.addDynamicCircle(
        phys2d::CircleParams{.pos = {400.f, 500.f}, .radius = 20.f, .friction = 0.8f});
    auto const hit = world.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f);
    check(hit.hit, "World::raycast acierta hacia abajo");
    check(hit.bodyId == 1, "World::raycast identifica el cuerpo");
    check(hit.point.y > 470.f && hit.point.y < 490.f, "World::raycast punto en la superficie");
}

void testDistanceJointHangs() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    int const anchor =
        world.addStaticBox(phys2d::BoxParams{.pos = {400.f, 100.f}, .half = {30.f, 10.f}});
    int const b =
        world.addDynamicCircle(phys2d::CircleParams{.pos = {400.f, 130.f}, .radius = 10.f});
    world.addJoint<phys2d::Distance>(anchor, b, 30.f);
    for (int i = 0; i < 240; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    check(near(body.pos.x, 400.f, 3.f) && near(body.pos.y, 130.f, 4.f),
          "Distance joint cuelga el cuerpo a rest length");
}

void testSpringReachesRestLength() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    int const anchor =
        world.addStaticBox(phys2d::BoxParams{.pos = {100.f, 100.f}, .half = {10.f, 10.f}});
    int const b =
        world.addDynamicCircle(phys2d::CircleParams{.pos = {200.f, 100.f}, .radius = 10.f});
    world.addJoint<phys2d::Spring>(anchor, b, 50.f, 100.f, 1.f);
    for (int i = 0; i < 600; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    check(near(body.pos.x, 150.f, 5.f), "Spring lleva el cuerpo a rest length");
}

void testCapsuleAndPolygonFall() {
    phys2d::World world = floorWorld();
    int const cap = world.addDynamicCapsule(phys2d::CapsuleParams{
        .pos = {200.f, 100.f}, .radius = 8.f, .length = 24.f, .friction = 0.8f});
    std::vector<phys2d::Vec2> tri{{-20.f, -10.f}, {20.f, -10.f}, {0.f, 20.f}};
    int const poly = world.addDynamicPolygon(
        phys2d::PolygonParams{.pos = {500.f, 100.f}, .verts = tri, .friction = 0.8f});
    for (int i = 0; i < 360; ++i) {
        world.step(1.f / 60.f);
    }
    check(world.body(cap).pos.y > 540.f, "cápsula cae al suelo");
    check(world.body(poly).pos.y > 540.f, "polígono cae al suelo");
}

void testRopeStopsSeparation() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    int const anchor =
        world.addStaticBox(phys2d::BoxParams{.pos = {0.f, 0.f}, .half = {10.f, 10.f}});
    int const b = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {30.f, 0.f}, .radius = 10.f, .vel = {100.f, 0.f}});
    (void)world.addRopeJoint(anchor, b, 50.f);
    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    check(body.pos.x < 70.f, "Rope frena la separación en maxLength");
    check(std::isfinite(body.vel.x) && std::abs(body.vel.x) < 200.f,
          "Rope no explota (velocidad acotada)");
}

void testGearKeepsRatio() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    int const a = world.addDynamicBox(phys2d::BoxParams{.pos = {0.f, 0.f}, .half = {20.f, 20.f}});
    int const b = world.addDynamicBox(phys2d::BoxParams{.pos = {200.f, 0.f}, .half = {20.f, 20.f}});
    world.body(a).angVel = 3.f;
    (void)world.addGearJoint(a, b, -2.f);
    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& bodyA = world.body(a);
    phys2d::Body const& bodyB = world.body(b);
    float const invariant = bodyB.angVel + (-2.f) * bodyA.angVel;
    check(std::abs(invariant) < 0.2f, "Gear mantiene angB + ratio*angA ≈ 0");
    check(std::abs(bodyA.angVel) < 10.f && std::abs(bodyB.angVel) < 10.f, "Gear no explota");
}

void testPulleyKeepsTotalLength() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    int const a = world.addDynamicBox(
        phys2d::BoxParams{.pos = {300.f, 200.f}, .half = {15.f, 15.f}, .vel = {0.f, 20.f}});
    int const b =
        world.addDynamicBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {15.f, 15.f}});
    (void)world.addPulleyJoint(a, b, {0.f, 0.f}, {0.f, 0.f}, {300.f, 0.f}, {500.f, 0.f}, 1.f);
    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& bodyA = world.body(a);
    phys2d::Body const& bodyB = world.body(b);
    float const lenA = (bodyA.pos - phys2d::Vec2{300.f, 0.f}).length();
    float const lenB = (bodyB.pos - phys2d::Vec2{500.f, 0.f}).length();
    check(std::abs((lenA + lenB) - 500.f) < 8.f, "Pulley conserva la longitud total");
    check(bodyA.pos.y > 200.f && bodyB.pos.y < 300.f,
          "Pulley transfiere movimiento: A baja y B sube");
    check(std::isfinite(bodyA.vel.y) && std::abs(bodyA.vel.y) < 100.f, "Pulley no explota");
}

void testJointHelpersAcceptBraces() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    int const a =
        world.addStaticBox(phys2d::BoxParams{.pos = {400.f, 100.f}, .half = {10.f, 10.f}});
    int const b = world.addDynamicBox(
        phys2d::BoxParams{.pos = {400.f, 200.f}, .half = {20.f, 20.f}, .linearDamping = 0.1f});
    // Sin phys2d::Vec2{} explícito: los helpers aceptan {0, 0} a pelo.
    (void)world.addRevoluteJoint(a, b, {0.f, 0.f}, {0.f, -100.f});
    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    float const dist = (body.pos - phys2d::Vec2{400.f, 100.f}).length();
    check(std::abs(dist - 100.f) < 5.f, "Revolute (helper con braces) mantiene la distancia");
}

void testCopyWorldKeepsJoints() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    int const anchor =
        world.addStaticBox(phys2d::BoxParams{.pos = {400.f, 100.f}, .half = {30.f, 10.f}});
    int const b =
        world.addDynamicCircle(phys2d::CircleParams{.pos = {400.f, 130.f}, .radius = 10.f});
    (void)world.addDistanceJoint(anchor, b, 30.f);
    phys2d::World copy = world;
    check(copy.jointCount() == world.jointCount() && copy.jointCount() == 1U,
          "copiar World clona los joints");
    for (int i = 0; i < 120; ++i) {
        copy.step(1.f / 60.f);
    }
    phys2d::Body const& body = copy.body(b);
    check(near(body.pos.x, 400.f, 3.f) && near(body.pos.y, 130.f, 4.f),
          "World clonado cuelga el cuerpo igual que el original");
    phys2d::World assigned;
    assigned = world;
    check(assigned.jointCount() == 1U, "asignar World también clona los joints");
}

void testShapeCastHitsCircle() {
    phys2d::RayHit const hit =
        phys2d::shapeCastCircle({0.f, 0.f}, {1.f, 0.f}, 100.f, 10.f, {50.f, 0.f}, 10.f);
    check(hit.hit && near(hit.fraction, 0.3f, 0.01f), "shapeCastCircle: contacto a fraction 0.30");
    phys2d::RayHit const miss =
        phys2d::shapeCastCircle({0.f, 0.f}, {1.f, 0.f}, 100.f, 10.f, {50.f, 90.f}, 10.f);
    check(!miss.hit, "shapeCastCircle: sin hit lejos del camino");
}

void testBulletExtremeSpeed() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {2.f, 200.f}});
    int const c = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {100.f, 300.f}, .radius = 10.f, .vel = {50000.f, 0.f}});
    world.body(c).bullet = true;
    for (int i = 0; i < 30; ++i) {
        world.step(1.f / 60.f);
    }
    check(world.body(c).pos.x < 510.f, "swept CCD detiene bala a 50000 px/s");
}

void testInvalidBodyThrows() {
    phys2d::World world = floorWorld();
    bool threw = false;
    try {
        (void)world.body(999);
    } catch (std::out_of_range const&) {
        threw = true;
    }
    check(threw, "body(id inválido) lanza std::out_of_range, no devuelve dummy");
}

void testBulletDoesNotTunnel() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {2.f, 200.f}});
    int const c = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {100.f, 300.f}, .radius = 10.f, .vel = {12000.f, 0.f}});
    world.body(c).bullet = true;
    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60.f);
    }
    check(world.body(c).pos.x < 510.f, "bala no tunela a través de pared fina");
}

void testBulletBoxStops() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {2.f, 200.f}});
    int const b = world.addDynamicBox(phys2d::BoxParams{
        .pos = {100.f, 300.f}, .half = {20.f, 20.f}, .vel = {20000.f, 0.f}, .restitution = 0.f});
    world.body(b).bullet = true;
    for (int i = 0; i < 60; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    std::println("bulletBox pos.x={} vel.x={}", body.pos.x, body.vel.x);
    check(body.pos.x < 520.f, "bala caja se detiene ante pared fina (pos.x < 520)");
    check(std::abs(body.vel.x) < 100.f, "bala caja frena |vel.x| < 100");
}

void testBulletPolygonStops() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {2.f, 200.f}});
    std::vector<phys2d::Vec2> tri{{-20.f, -20.f}, {20.f, 0.f}, {-20.f, 20.f}};
    int const p = world.addDynamicPolygon(phys2d::PolygonParams{
        .pos = {100.f, 300.f}, .verts = tri, .vel = {20000.f, 0.f}, .restitution = 0.f});
    world.body(p).bullet = true;
    for (int i = 0; i < 60; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(p);
    std::println("bulletPolygon pos.x={} vel=({},{})", body.pos.x, body.vel.x, body.vel.y);
    check(body.pos.x < 520.f, "bala polígono se detiene ante pared fina (pos.x < 520)");
}

void testBulletCapsuleStops() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {2.f, 200.f}});
    int const c = world.addDynamicCapsule(phys2d::CapsuleParams{.pos = {100.f, 300.f},
                                                                .radius = 10.f,
                                                                .length = 40.f,
                                                                .vel = {20000.f, 0.f},
                                                                .restitution = 0.f});
    world.body(c).bullet = true;
    for (int i = 0; i < 60; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(c);
    std::println("bulletCapsule pos.x={} vel.len={}", body.pos.x, body.vel.length());
    check(body.pos.x < 520.f, "bala cápsula se detiene ante pared fina (pos.x < 520)");
    check(body.vel.length() < 500.f, "bala cápsula |vel| acotada (< 500)");
}

void testBulletRotatedBoxStops() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {2.f, 200.f}});
    int const b = world.addDynamicBox(phys2d::BoxParams{
        .pos = {100.f, 300.f}, .half = {20.f, 20.f}, .vel = {20000.f, 0.f}, .angle = 0.5f});
    world.body(b).bullet = true;
    for (int i = 0; i < 60; ++i) {
        world.step(1.f / 60.f);
    }
    phys2d::Body const& body = world.body(b);
    std::println("bulletRotatedBox pos.x={} vel=({},{})", body.pos.x, body.vel.x, body.vel.y);
    check(body.pos.x < 520.f, "bala caja rotada no tunela (pos.x < 520)");
    check(std::isfinite(body.pos.x) && std::isfinite(body.vel.length()),
          "bala caja rotada sigue finita tras el impacto");
}

void testRevoluteMotorDrivesRadiansPerSecond() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    int const anchor =
        world.addStaticBox(phys2d::BoxParams{.pos = {0.f, 0.f}, .half = {10.f, 10.f}});
    int const b = world.addDynamicBox(
        phys2d::BoxParams{.pos = {100.f, 0.f}, .half = {20.f, 20.f}, .mass = 2.f});
    int const j = world.addRevoluteJoint(anchor, b, {0.f, 0.f}, {-100.f, 0.f});
    auto& rev = static_cast<phys2d::Revolute&>(world.joint(j));
    rev.enableMotor = true;
    rev.motorSpeed = 2.f; // rad/s
    rev.maxMotorTorque = 500.f;
    for (int i = 0; i < 240; ++i) {
        world.step(1.f / 60.f);
    }
    float const wB = world.body(b).angVel;
    check(near(wB, 2.f, 0.5f), "motor Revolute alcanza motorSpeed en rad/s (no 60x)");
}

void testThickSegmentCollidesWithCircle() {
    phys2d::Body ball;
    ball.pos = {0.f, 0.f};
    ball.shape = phys2d::Circle{10.f};
    phys2d::Body seg;
    seg.pos = {24.f, 0.f};
    seg.shape = phys2d::Segment{.a = {0.f, -100.f}, .b = {0.f, 100.f}, .radius = 15.f};
    phys2d::Manifold manifold{};
    check(phys2d::collide(ball, seg, manifold), "círculo toca segmento grueso a dist 24 < 25");
    check(manifold.depth > 0.f && manifold.count >= 1, "segmento grueso reporta profundidad");
    phys2d::Body thin;
    thin.pos = {24.f, 0.f};
    thin.shape = phys2d::Segment{.a = {0.f, -100.f}, .b = {0.f, 100.f}, .radius = 0.f};
    phys2d::Manifold thinM{};
    check(!phys2d::collide(ball, thin, thinM), "segmento fino a distancia 24 no colisiona");
}

void testThickParallelSegmentsCollide() {
    phys2d::Body a;
    a.pos = {0.f, 0.f};
    a.shape = phys2d::Segment{.a = {-90.f, 0.f}, .b = {90.f, 0.f}, .radius = 10.f};
    phys2d::Body b;
    b.pos = {0.f, 15.f};
    b.shape = phys2d::Segment{.a = {-90.f, 0.f}, .b = {90.f, 0.f}, .radius = 10.f};
    phys2d::Manifold manifold{};
    check(phys2d::collide(a, b, manifold) && manifold.depth > 0.f,
          "segmentos gruesos paralelos solapados colisionan");
    phys2d::Body thinA;
    thinA.pos = {0.f, 0.f};
    thinA.shape = phys2d::Segment{.a = {-90.f, 0.f}, .b = {90.f, 0.f}, .radius = 0.f};
    phys2d::Body thinB;
    thinB.pos = {0.f, 1.f};
    thinB.shape = phys2d::Segment{.a = {-90.f, 0.f}, .b = {90.f, 0.f}, .radius = 0.f};
    phys2d::Manifold thinM{};
    check(!phys2d::collide(thinA, thinB, thinM),
          "segmentos finos paralelos solapados no colisionan");
}

void testAddJointValidatesIds() {
    phys2d::World world;
    int const a = world.addStaticBox(phys2d::BoxParams{.pos = {0.f, 0.f}, .half = {10.f, 10.f}});
    int const b = world.addDynamicCircle(phys2d::CircleParams{.pos = {50.f, 0.f}, .radius = 10.f});
    bool outOfRange = false;
    try {
        (void)world.addJoint<phys2d::Distance>(a, 999, 10.f);
    } catch (std::out_of_range const&) {
        outOfRange = true;
    }
    check(outOfRange, "addJoint con id fuera de rango lanza std::out_of_range");
    bool sameIndex = false;
    try {
        (void)world.addJoint<phys2d::Distance>(a, a, 10.f);
    } catch (std::invalid_argument const&) {
        sameIndex = true;
    }
    check(sameIndex, "addJoint con a == b lanza std::invalid_argument");
    bool negativeId = false;
    try {
        (void)world.addJoint<phys2d::Distance>(-1, b, 10.f);
    } catch (std::out_of_range const&) {
        negativeId = true;
    }
    check(negativeId, "addJoint con id negativo lanza std::out_of_range");
    bool ok = false;
    try {
        (void)world.addDistanceJoint(a, b, 20.f);
        ok = true;
    } catch (...) {
    }
    check(ok, "addJoint con ids válidos no lanza");
}

void testMoveAssignedWorldSteps() {
    phys2d::World original;
    original.gravity = {0.f, 900.f};
    (void)original.addStaticBox(phys2d::BoxParams{.pos = {100.f, 100.f}, .half = {50.f, 10.f}});
    (void)original.addDynamicCircle(phys2d::CircleParams{.pos = {100.f, 0.f}, .radius = 10.f});
    phys2d::World moved;
    moved = std::move(original);
    for (int i = 0; i < 30; ++i) {
        moved.step(1.f / 60.f);
    }
    check(moved.size() == 2U, "World asignado por movimiento sigue simulando");
    check(std::isfinite(moved.body(1).pos.y), "World movido: posición finita tras step()");
}

void testInvalidFrictionAndRestitutionThrow() {
    phys2d::World world;
    bool badRestitution = false;
    try {
        (void)world.addDynamicCircle(
            phys2d::CircleParams{.pos = {0.f, 0.f}, .radius = 10.f, .restitution = 2.f});
    } catch (std::invalid_argument const&) {
        badRestitution = true;
    }
    check(badRestitution, "restitution > 1 lanza std::invalid_argument");
    bool badFriction = false;
    try {
        (void)world.addDynamicCircle(
            phys2d::CircleParams{.pos = {0.f, 0.f}, .radius = 10.f, .friction = -0.5f});
    } catch (std::invalid_argument const&) {
        badFriction = true;
    }
    check(badFriction, "friction < 0 lanza std::invalid_argument");
    bool badNegativeRestitution = false;
    try {
        (void)world.addDynamicCircle(
            phys2d::CircleParams{.pos = {0.f, 0.f}, .radius = 10.f, .restitution = -0.1f});
    } catch (std::invalid_argument const&) {
        badNegativeRestitution = true;
    }
    check(badNegativeRestitution, "restitution < 0 lanza std::invalid_argument");
}

void testNaNPolygonVertexThrows() {
    phys2d::World world;
    float const inf = std::numeric_limits<float>::infinity();
    bool threw = false;
    try {
        (void)world.addDynamicPolygon(phys2d::PolygonParams{
            .pos = {0.f, 0.f}, .verts = {{0.f, 0.f}, {inf, 0.f}, {0.f, 10.f}}});
    } catch (std::invalid_argument const&) {
        threw = true;
    }
    check(threw, "polígono con vértice inf lanza std::invalid_argument");
}

void testCapsuleDirValidated() {
    phys2d::World world;
    bool zeroDir = false;
    try {
        (void)world.addDynamicCapsule(phys2d::CapsuleParams{
            .pos = {0.f, 0.f}, .radius = 5.f, .length = 20.f, .dir = {0.f, 0.f}});
    } catch (std::invalid_argument const&) {
        zeroDir = true;
    }
    check(zeroDir, "cápsula con dir nulo lanza std::invalid_argument");
    int const id = world.addDynamicCapsule(
        phys2d::CapsuleParams{.pos = {0.f, 0.f}, .radius = 5.f, .length = 20.f, .dir = {3.f, 4.f}});
    auto const& cap = std::get<phys2d::Capsule>(world.body(id).shape);
    check(near(cap.dir.length(), 1.f, 1e-6f), "cápsula con dir no normalizada se normaliza");
}

void testStepWithInvalidDtThrows() {
    phys2d::World world;
    bool zeroDt = false;
    try {
        world.step(0.f);
    } catch (std::invalid_argument const&) {
        zeroDt = true;
    }
    check(zeroDt, "step(0) lanza std::invalid_argument");
    bool nanDt = false;
    try {
        world.step(std::numeric_limits<float>::quiet_NaN());
    } catch (std::invalid_argument const&) {
        nanDt = true;
    }
    check(nanDt, "step(NaN) lanza std::invalid_argument");
}

void testRaycastHitsExactMaxDist() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};
    (void)world.addDynamicCircle(
        phys2d::CircleParams{.pos = {50.f, 0.f}, .radius = 10.f, .vel = {0.f, 0.f}});
    auto const hit = world.raycast({0.f, 0.f}, {1.f, 0.f}, 40.f);
    check(hit.hit, "raycast acierta un cuerpo cuya superficie está a maxDist exacto");
    check(near(hit.fraction, 1.f, 0.01f), "raycast en maxDist exacto da fracción ≈ 1");
}

void testMat2Transpose() {
    phys2d::Mat2 const m{1.f, 2.f, 3.f, 4.f};
    phys2d::Mat2 const t = m.transpose();
    check(t.col0 == phys2d::Vec2{1.f, 2.f} && t.col1 == phys2d::Vec2{3.f, 4.f},
          "Mat2::transpose intercambia filas y columnas");
    phys2d::Mat2 const r = phys2d::Mat2::rotation(0.3f);
    phys2d::Mat2 const rt = r.transpose();
    phys2d::Mat2 const rInv = phys2d::Mat2::rotation(-0.3f);
    check(near(rt.col0.x, rInv.col0.x, 1e-6f) && near(rt.col1.x, rInv.col1.x, 1e-6f),
          "transpose de rotación coincide con la rotación inversa");
}

void testStaticStaticDoesNotFireBeginCallbacks() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {100.f, 100.f}, .half = {30.f, 30.f}});
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {100.f, 100.f}, .half = {20.f, 20.f}});
    int begins = 0;
    world.onBeginContact([&begins](phys2d::Manifold const&) { ++begins; });
    for (int i = 0; i < 30; ++i) {
        world.step(1.f / 60.f);
    }
    check(begins == 0, "pares estático-estático no disparan onBeginContact");
}

void testBeginCallbackReceivesValidManifold() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    (void)world.addStaticBox(phys2d::BoxParams{.pos = {100.f, 590.f}, .half = {200.f, 10.f}});
    int const ball =
        world.addDynamicCircle(phys2d::CircleParams{.pos = {100.f, 550.f}, .radius = 20.f});
    int begins = 0;
    int bodiesSeen = 0;
    world.onBeginContact([&begins, &bodiesSeen, ball](phys2d::Manifold const& c) {
        ++begins;
        if ((c.a == 0 && c.b == ball) || (c.b == 0 && c.a == ball)) {
            bodiesSeen = c.a + c.b;
        }
        check(std::isfinite(c.normal.x) && std::isfinite(c.normal.y) && std::isfinite(c.depth),
              "callback recibe normal/depth finitos");
    });
    for (int i = 0; i < 240; ++i) {
        world.step(1.f / 60.f);
    }
    check(begins >= 1, "onBeginContact se dispara al caer la pelota al suelo");
    check(bodiesSeen == 0 + ball, "callback identifica el par correcto");
}

void testDegenerateShapesThrow() {
    phys2d::World world;
    bool badCircle = false;
    try {
        (void)world.addDynamicCircle(phys2d::CircleParams{.pos = {0.f, 0.f}, .radius = -5.f});
    } catch (std::invalid_argument const&) {
        badCircle = true;
    }
    check(badCircle, "círculo con radio negativo lanza std::invalid_argument");

    bool badPolygon = false;
    try {
        (void)world.addDynamicPolygon(
            phys2d::PolygonParams{.pos = {0.f, 0.f}, .verts = {{0.f, 0.f}, {10.f, 0.f}}});
    } catch (std::invalid_argument const&) {
        badPolygon = true;
    }
    check(badPolygon, "polígono de 2 vértices lanza std::invalid_argument");
}

void testShapePairMatrix() {
    // 5 formas -> 15 pares no ordenados con repetición (i <= j): los 10
    // pares distintos más los 5 pares de la misma forma consigo misma.
    // A en {0,0} y B en {10,0}: solapados en el origen, pero con centros
    // distintos para que dot(normal, posB-posA) >= 0 sea comprobable.
    // Segment con radius > 0 (cápsula fina); collide() interpreta los
    // segmentos como de grosor cero en sus pares, pero la línea cruza
    // el otro cuerpo, así que debe colisionar igualmente.
    std::array<phys2d::Shape, 5> const shapes{
        phys2d::Shape{phys2d::Circle{.radius = 20.f}},
        phys2d::Shape{phys2d::AABB::fromCenterHalf({0.f, 0.f}, {20.f, 20.f})},
        phys2d::Shape{phys2d::ConvexPolygon{
            .verts = {{-20.f, -20.f}, {20.f, -20.f}, {20.f, 20.f}, {-20.f, 20.f}}}},
        phys2d::Shape{phys2d::Capsule{.radius = 10.f, .length = 30.f, .dir = {0.f, 1.f}}},
        phys2d::Shape{phys2d::Segment{.a = {-30.f, 0.f}, .b = {30.f, 0.f}, .radius = 5.f}}};
    std::array<char const*, 5> const names{"Circle", "AABB/Box", "ConvexPolygon", "Capsule",
                                           "Segment"};
    auto makeBody = [](phys2d::Shape const& shape, phys2d::Vec2 pos) {
        phys2d::Body body;
        body.pos = pos;
        body.shape = shape;
        body.vel = {0.f, 0.f};
        return body;
    };
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        for (std::size_t j = i; j < shapes.size(); ++j) {
            phys2d::Body a = makeBody(shapes[i], {0.f, 0.f});
            phys2d::Body b = makeBody(shapes[j], {10.f, 0.f});
            phys2d::Manifold manifold{};
            bool const hit = phys2d::collide(a, b, manifold);
            phys2d::Vec2 const delta = b.pos - a.pos;
            bool const finite = std::isfinite(manifold.normal.x) &&
                                std::isfinite(manifold.normal.y) && std::isfinite(manifold.depth);
            bool const ok =
                hit && finite && manifold.depth >= 0.f && manifold.normal.dot(delta) >= 0.f;
            if (!ok) {
                std::println("  par {}-{}: hit={} depth={} normal=({},{}) dot={}", names[i],
                             names[j], hit, manifold.depth, manifold.normal.x, manifold.normal.y,
                             manifold.normal.dot(delta));
            }
            check(ok, "matriz de pares de formas: colisión solapada con normal A->B");
        }
    }
}

void testNonFiniteInputsThrow() {
    float const nan = std::numeric_limits<float>::quiet_NaN();
    float const inf = std::numeric_limits<float>::infinity();

    bool badCirclePos = false;
    try {
        phys2d::World world;
        (void)world.addDynamicCircle(phys2d::CircleParams{.pos = {nan, 0.f}, .radius = 20.f});
    } catch (std::invalid_argument const&) {
        badCirclePos = true;
    }
    check(badCirclePos, "círculo con pos {NaN,0} lanza std::invalid_argument");

    bool badCircleVel = false;
    try {
        phys2d::World world;
        (void)world.addDynamicCircle(
            phys2d::CircleParams{.pos = {0.f, 0.f}, .radius = 20.f, .vel = {inf, 0.f}});
    } catch (std::invalid_argument const&) {
        badCircleVel = true;
    }
    check(badCircleVel, "círculo con vel {inf,0} lanza std::invalid_argument");

    bool badCircleRadius = false;
    try {
        phys2d::World world;
        (void)world.addDynamicCircle(phys2d::CircleParams{.pos = {0.f, 0.f}, .radius = nan});
    } catch (std::invalid_argument const&) {
        badCircleRadius = true;
    }
    check(badCircleRadius, "círculo con radio NaN lanza std::invalid_argument");

    bool badBoxPos = false;
    try {
        phys2d::World world;
        (void)world.addDynamicBox(phys2d::BoxParams{.pos = {0.f, inf}, .half = {20.f, 20.f}});
    } catch (std::invalid_argument const&) {
        badBoxPos = true;
    }
    check(badBoxPos, "caja con pos {0,inf} lanza std::invalid_argument");
}

} // namespace

int main() {
    testCircleRestsOnFloor();
    testRotatedBoxFlattens();
    testStackOfFourBoxes();
    testSettledStackSleeps();
    testWorldRaycast();
    testRevoluteMotorDrivesRadiansPerSecond();
    testThickSegmentCollidesWithCircle();
    testThickParallelSegmentsCollide();
    testAddJointValidatesIds();
    testMoveAssignedWorldSteps();
    testInvalidFrictionAndRestitutionThrow();
    testNaNPolygonVertexThrows();
    testCapsuleDirValidated();
    testStepWithInvalidDtThrows();
    testRaycastHitsExactMaxDist();
    testMat2Transpose();
    testStaticStaticDoesNotFireBeginCallbacks();
    testBeginCallbackReceivesValidManifold();
    testDistanceJointHangs();
    testSpringReachesRestLength();
    testCapsuleAndPolygonFall();
    testRopeStopsSeparation();
    testGearKeepsRatio();
    testPulleyKeepsTotalLength();
    testJointHelpersAcceptBraces();
    testCopyWorldKeepsJoints();
    testShapeCastHitsCircle();
    testBulletExtremeSpeed();
    testInvalidBodyThrows();
    testDegenerateShapesThrow();
    testShapePairMatrix();
    testNonFiniteInputsThrow();
    testBulletDoesNotTunnel();
    testBulletBoxStops();
    testBulletPolygonStops();
    testBulletCapsuleStops();
    testBulletRotatedBoxStops();
    if (failures == 0) {
        std::println("tests: OK");
        return 0;
    }
    std::println("tests: {} fallos", failures);
    return 1;
}
