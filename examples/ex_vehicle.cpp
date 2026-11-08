#include "phys2d/phys2d.hpp"
#include <cmath>
#include <print>

int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    int const ground = world.addStaticBox(
        phys2d::BoxParams{.pos = {400.f, 590.f}, .half = {500.f, 10.f}, .friction = 0.9f});
    (void)ground;

    constexpr float startX = 180.f;
    constexpr float axleOffset = 38.f;
    constexpr float wheelY = 34.f;
    int const chassis = world.addDynamicBox(phys2d::BoxParams{.pos = {startX, 520.f},
                                                              .half = {58.f, 14.f},
                                                              .mass = 4.f,
                                                              .friction = 0.6f,
                                                              .linearDamping = 0.05f,
                                                              .angularDamping = 0.1f});

    auto addWheel = [&world](float x) {
        return world.addDynamicCircle(phys2d::CircleParams{.pos = {x, 554.f},
                                                           .radius = 20.f,
                                                           .mass = 1.f,
                                                           .friction = 1.f,
                                                           .linearDamping = 0.02f,
                                                           .angularDamping = 0.02f});
    };
    int const rearWheel = addWheel(startX - axleOffset);
    int const frontWheel = addWheel(startX + axleOffset);

    int const rearAxle = world.addRevoluteJoint(chassis, rearWheel, {-axleOffset, wheelY}, {});
    int const frontAxle = world.addRevoluteJoint(chassis, frontWheel, {axleOffset, wheelY}, {});
    for (int axle : {rearAxle, frontAxle}) {
        auto& motor = static_cast<phys2d::Revolute&>(world.joint(axle));
        motor.enableMotor = true;
        motor.motorSpeed = 8.f;
        motor.maxMotorTorque = 1200.f;
    }

    for (int i = 0; i < 300; ++i) {
        world.step(1.f / 60.f);
    }

    phys2d::Body const& body = world.body(chassis);
    phys2d::Body const& rear = world.body(rearWheel);
    phys2d::Body const& front = world.body(frontWheel);
    float const distance = body.pos.x - startX;
    std::println("ex_vehicle: powered two-wheel car");
    std::println("  chassis pos=({:.1f}, {:.1f}) angle={:.2f}", body.pos.x, body.pos.y, body.angle);
    std::println("  rear wheel pos=({:.1f}, {:.1f}) angle={:.2f}", rear.pos.x, rear.pos.y,
                 rear.angle);
    std::println("  front wheel pos=({:.1f}, {:.1f}) angle={:.2f}", front.pos.x, front.pos.y,
                 front.angle);
    std::println("  distance traveled={:.1f}  {}", distance,
                 std::isfinite(distance) && distance > 20.f ? "PASS" : "FAIL");
    return std::isfinite(distance) && distance > 20.f ? 0 : 1;
}
