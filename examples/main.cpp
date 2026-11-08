#include "phys2d/phys2d.hpp"
#include <print>

// API ultra fácil (C++23 no admite mezclar {{100,50}, .radius=20},
// así que se usan inicializadores designados completos):
//   w.addStaticBox({.pos={400,590}, .half={500,10}});
//   w.addDynamicCircle({.pos={100,50}, .radius=20, .vel={40,0}});
//   w.addJoint<Distance>(a, b, 150.f);
//   auto hit = w.raycast({400,0}, {0,1}, 600.f);
//   for (auto& b : w) ...
int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    int const floorId =
        world.addStaticBox(phys2d::BoxParams{.pos = {400.f, 590.f}, .half = {500.f, 10.f}});
    world.body(floorId).friction = 0.8f;

    int const b1 = world.addDynamicCircle(phys2d::CircleParams{.pos = {100.f, 50.f},
                                                               .radius = 20.f,
                                                               .vel = {10.f, 0.f},
                                                               .friction = 0.5f,
                                                               .linearDamping = 0.2f,
                                                               .angularDamping = 0.2f});
    int const b2 = world.addDynamicCircle(phys2d::CircleParams{.pos = {200.f, 30.f},
                                                               .radius = 15.f,
                                                               .vel = {-20.f, 0.f},
                                                               .friction = 0.5f,
                                                               .linearDamping = 0.2f,
                                                               .angularDamping = 0.2f});
    int const box = world.addDynamicBox(phys2d::BoxParams{.pos = {400.f, 40.f},
                                                          .half = {20.f, 20.f},
                                                          .angle = 0.3f,
                                                          .friction = 0.5f,
                                                          .linearDamping = 0.2f,
                                                          .angularDamping = 0.2f});

    std::println("inicio: {} cuerpos", world.size());
    for (int i = 0; i < 180; ++i) {
        world.step(1.f / 60.f);
    }

    for (auto& body : world) {
        std::println("pos={:.1f},{:.1f} vel={:.1f},{:.1f} angle={:.2f}", body.pos.x, body.pos.y,
                     body.vel.x, body.vel.y, body.angle);
    }
    std::println("contactos: {}", world.contacts().size());

    auto const hit = world.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f);
    std::println("raycast down hit={} body={} frac={:.3f}", hit.hit, hit.bodyId, hit.fraction);
    (void)b1;
    (void)b2;
    (void)box;
    (void)floorId;
    return 0;
}
