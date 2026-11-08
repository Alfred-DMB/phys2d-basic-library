#include "phys2d/phys2d.hpp"
#include <array>
#include <cmath>
#include <print>

namespace {

struct TextDraw : phys2d::DebugDraw {
    void drawCircle(phys2d::Vec2 center, float radius) override {
        std::println("  drawCircle c={:.1f},{:.1f} r={:.1f}", center.x, center.y, radius);
    }
    void drawBox(phys2d::Vec2 center, phys2d::Vec2 half, float angle) override {
        std::println("  drawBox c={:.1f},{:.1f} half={:.1f},{:.1f} a={:.2f}", center.x, center.y,
                     half.x, half.y, angle);
    }
    void drawSegment(phys2d::Vec2 a, phys2d::Vec2 b) override {
        std::println("  drawSegment {:.1f},{:.1f} -> {:.1f},{:.1f}", a.x, a.y, b.x, b.y);
    }
};

} // namespace

// API soñada (PLAN.md): w.addStaticBox({...}); w.addDynamicBox({...});
int main() {
    phys2d::World w;
    w.gravity = {0.f, 900.f};
    w.onBeginContact([](auto const& c) {
        std::println("  contacto a={} b={} depth={:.2f}", c.a, c.b, c.depth);
    });

    int const floorId = w.addStaticBox(
        phys2d::BoxParams{.pos = phys2d::Vec2{400.f, 590.f}, .half = phys2d::Vec2{500.f, 10.f}});
    w.body(floorId).friction = 0.8f;

    std::array<int, 4> ids{-1, -1, -1, -1};
    for (std::size_t i = 0U; i < ids.size(); ++i) {
        phys2d::BoxParams p{};
        p.pos = phys2d::Vec2{400.f + static_cast<float>(i % 2) * 1.f,
                             560.f - static_cast<float>(i) * 40.f - 200.f};
        p.half = phys2d::Vec2{20.f, 20.f};
        p.friction = 0.8f;
        ids[i] = w.addDynamicBox(p);
    }

    std::println("ex_stack: {} cuerpos, floor={}", w.size(), floorId);
    for (int i = 0; i < 360; ++i) {
        w.step(1.f / 60.f);
    }

    float maxDev = 0.f;
    for (std::size_t i = 0U; i < ids.size(); ++i) {
        phys2d::Body const& b = w.body(ids[i]);
        float const expectY = 560.f - static_cast<float>(i) * 40.f;
        float const dev = std::abs(b.pos.y - expectY);
        if (dev > maxDev)
            maxDev = dev;
        std::println("  box{} pos={:.2f},{:.2f} expectY={:.1f} dev={:.2f} angle={:.3f}", i, b.pos.x,
                     b.pos.y, expectY, dev, b.angle);
    }
    std::println("contactos: {}", w.contacts().size());
    std::println("maxDesviacionY={:.2f} {}", maxDev, maxDev <= 2.f ? "PASS" : "WARN>2px");

    TextDraw dd{};
    std::println("drawWorld:");
    phys2d::drawWorld(dd, w.bodiesSpan());

    auto hit = phys2d::raycastPoint(phys2d::Vec2{400.f, 0.f}, phys2d::Vec2{0.f, 1.f}, 600.f,
                                    w.bodiesSpan());
    std::println("raycast down hit={} body={} frac={:.3f} y={:.1f}", hit.hit, hit.bodyId,
                 hit.fraction, hit.point.y);
    for (auto& b : w) {
        std::println("  iter pos={:.1f},{:.1f}", b.pos.x, b.pos.y);
    }
    return 0;
}
