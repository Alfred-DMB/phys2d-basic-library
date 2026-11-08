#include "phys2d/phys2d.hpp"
#include <print>
#include <vector>

// Cadena de 8 eslabones colgando de un ancla estática.
// Los joints viven dentro de World: w.addJoint<Distance>(a, b, rest)
// y World::step los resuelve junto con los contactos.
int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    int const anchor =
        world.addStaticBox(phys2d::BoxParams{.pos = {400.f, 100.f}, .half = {30.f, 10.f}});
    std::println("ex_chain: anchor={}", anchor);

    std::vector<int> ids;
    ids.reserve(8U);
    for (int i = 0; i < 8; ++i) {
        ids.push_back(world.addDynamicCircle(
            phys2d::CircleParams{.pos = {400.f, 130.f + static_cast<float>(i) * 30.f},
                                 .radius = 10.f,
                                 .linearDamping = 0.05f,
                                 .angularDamping = 0.05f}));
    }

    float const rest = 30.f;
    world.addJoint<phys2d::Distance>(anchor, ids[0], rest);
    for (std::size_t i = 0U; i + 1U < ids.size(); ++i) {
        world.addJoint<phys2d::Distance>(ids[i], ids[i + 1U], rest);
    }

    for (int i = 0; i < 240; ++i) {
        world.step(1.f / 60.f);
    }

    for (std::size_t i = 0U; i < ids.size(); ++i) {
        phys2d::Body const& body = world.body(ids[i]);
        std::println("  link{} pos={:.1f},{:.1f} vel={:.1f},{:.1f}", i, body.pos.x, body.pos.y,
                     body.vel.x, body.vel.y);
    }
    phys2d::Body const& tail = world.body(ids.back());
    std::println("cola pos={:.1f},{:.1f} lenTotal~{:.1f}", tail.pos.x, tail.pos.y, 8.f * rest);
    return 0;
}
