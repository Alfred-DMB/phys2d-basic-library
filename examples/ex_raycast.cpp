#include "phys2d/debug.hpp"
#include "phys2d/phys2d.hpp"
#include "phys2d/query.hpp"
#include <cmath>
#include <print>
#include <vector>

namespace {

struct SilentDraw : phys2d::DebugDraw {
    void drawCircle(phys2d::Vec2, float) override {
    }
    void drawBox(phys2d::Vec2, phys2d::Vec2, float) override {
    }
    void drawSegment(phys2d::Vec2, phys2d::Vec2) override {
    }
};

} // namespace

int main() {
    phys2d::World w;
    w.gravity = {0.f, 900.f};

    int const floorId = w.addStaticBox(
        phys2d::BoxParams{.pos = phys2d::Vec2{400.f, 590.f}, .half = phys2d::Vec2{500.f, 10.f}});
    w.body(floorId).friction = 0.8f;
    int const ball = w.addDynamicCircle(phys2d::CircleParams{
        .pos = phys2d::Vec2{100.f, 50.f}, .radius = 20.f, .vel = phys2d::Vec2{40.f, 0.f}});
    int const box = w.addDynamicBox(phys2d::BoxParams{
        .pos = phys2d::Vec2{400.f, 40.f}, .half = phys2d::Vec2{20.f, 20.f}, .angle = 0.3f});
    int const caps = w.addDynamicCapsule(
        phys2d::CapsuleParams{.pos = phys2d::Vec2{200.f, 60.f}, .radius = 8.f, .length = 24.f});
    std::vector<phys2d::Vec2> tri{phys2d::Vec2{-20.f, -10.f}, phys2d::Vec2{20.f, -10.f},
                                  phys2d::Vec2{0.f, 20.f}};
    int const poly = w.addDynamicPolygon(phys2d::Vec2{500.f, 80.f}, tri, phys2d::Vec2{}, 1.f, 0.f);

    for (int i = 0; i < 120; ++i)
        w.step(1.f / 60.f);
    std::println("ex_raycast: cuerpos={} contactos={}", w.size(), w.contacts().size());

    // Ultra fácil: el raycast vive en World.
    auto hitDown = w.raycast(phys2d::Vec2{400.f, 0.f}, phys2d::Vec2{0.f, 1.f}, 600.f);
    std::println(
        "raycastPoint down hit={} body={} frac={:.3f} point={:.1f},{:.1f} normal={:.2f},{:.2f}",
        hitDown.hit, hitDown.bodyId, hitDown.fraction, hitDown.point.x, hitDown.point.y,
        hitDown.normal.x, hitDown.normal.y);

    auto hitSide = phys2d::raycastPoint(phys2d::Vec2{0.f, 50.f}, phys2d::Vec2{1.f, 0.f}, 800.f,
                                        w.bodiesSpan());
    std::println("raycastPoint side hit={} body={} frac={:.3f} point={:.1f},{:.1f}", hitSide.hit,
                 hitSide.bodyId, hitSide.fraction, hitSide.point.x, hitSide.point.y);

    auto hC = phys2d::raycastCircle(phys2d::Vec2{0.f, 0.f}, phys2d::Vec2{1.f, 0.f}, 100.f,
                                    phys2d::Vec2{50.f, 0.f}, 10.f);
    std::println("raycastCircle hit={} frac={:.2f} expect 0.40", hC.hit, hC.fraction);
    auto hA = phys2d::raycastAABB(phys2d::Vec2{0.f, 0.f}, phys2d::Vec2{1.f, 0.f}, 100.f,
                                  phys2d::Vec2{50.f, -10.f}, phys2d::Vec2{60.f, 10.f});
    std::println("raycastAABB hit={} frac={:.2f} expect 0.50", hA.hit, hA.fraction);
    std::vector<phys2d::Vec2> quad{phys2d::Vec2{40.f, -10.f}, phys2d::Vec2{60.f, -10.f},
                                   phys2d::Vec2{60.f, 10.f}, phys2d::Vec2{40.f, 10.f}};
    auto hP = phys2d::raycastPolygon(phys2d::Vec2{0.f, 0.f}, phys2d::Vec2{1.f, 0.f}, 100.f,
                                     std::span<phys2d::Vec2 const>{quad});
    std::println("raycastPolygon hit={} frac={:.2f} expect 0.40", hP.hit, hP.fraction);
    auto hK = phys2d::raycastCapsule(phys2d::Vec2{0.f, 0.f}, phys2d::Vec2{1.f, 0.f}, 100.f,
                                     phys2d::Vec2{50.f, -30.f}, phys2d::Vec2{50.f, 30.f}, 10.f);
    std::println("raycastCapsule hit={} frac={:.2f} expect 0.40", hK.hit, hK.fraction);

    phys2d::Body const& ballBody = w.body(ball);
    bool const in = phys2d::pointInShape(ballBody.pos, ballBody);
    phys2d::Vec2 closest = phys2d::closestPointOnShape(phys2d::Vec2{0.f, 0.f}, ballBody);
    std::println("pointInShape(centro)={} closestDesdeOrigen={:.1f},{:.1f}", in, closest.x,
                 closest.y);

    auto a = phys2d::bodyAABB(ballBody);
    auto b = phys2d::bodyAABB(w.body(box));
    std::println("overlapAABB(ball,box)={}", phys2d::overlapAABB(a, b));
    SilentDraw sd{};
    phys2d::drawWorld(sd, w.bodiesSpan());
    std::println("drawWorld+bodiesSpan: ok ({} cuerpos, bodiesSpan={})", w.size(),
                 w.bodiesSpan().size());
    (void)caps;
    (void)poly;
    return 0;
}
