#include "visual_common.hpp"
#include <array>
#include <cmath>
#include <print>

namespace {

constexpr int terrainSegments = 50;

void drawShock(phys2d::Vec2 top, phys2d::Vec2 bottom) {
    using namespace phys2d;
    Vec2 const axis = (bottom - top).normalizedOr({0.f, 1.f});
    Vec2 const side = axis.perpendicular();
    constexpr int coils = 7;
    constexpr float width = 6.f;
    Vector2 previous = visual::toRaylib(top);

    DrawLineEx(previous, visual::toRaylib(bottom), 3.f, Color{38, 48, 62, 255});
    for (int i = 1; i < coils * 2; ++i) {
        float const t = static_cast<float>(i) / static_cast<float>(coils * 2);
        float const offset = i % 2 == 0 ? -width : width;
        Vector2 const point = visual::toRaylib(top + (bottom - top) * t + side * offset);
        DrawLineEx(previous, point, 3.f, Color{245, 180, 63, 255});
        previous = point;
    }
    DrawLineEx(previous, visual::toRaylib(bottom), 3.f, Color{245, 180, 63, 255});
    DrawCircleV(visual::toRaylib(top), 5.f, Color{38, 48, 62, 255});
    DrawCircleV(visual::toRaylib(bottom), 5.f, Color{38, 48, 62, 255});
}

phys2d::World makeWorld() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    for (int i = 0; i < terrainSegments; ++i) {
        float const x = -200.f + static_cast<float>(i) * 100.f;
        float const nextX = x + 100.f;
        auto height = [](float position) {
            return 590.f + 24.f * std::sin(position / 320.f) + 10.f * std::sin(position / 115.f);
        };
        (void)world.addStaticSegment(phys2d::SegmentParams{
            .a = {x, height(x)}, .b = {nextX, height(nextX)}, .radius = 10.f, .friction = 1.f});
    }

    constexpr float startX = 180.f;
    constexpr float axleOffset = 38.f;
    int const chassis = world.addDynamicBox(phys2d::BoxParams{.pos = {startX, 520.f},
                                                              .half = {58.f, 14.f},
                                                              .mass = 4.f,
                                                              .friction = 0.7f,
                                                              .linearDamping = 0.04f,
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
    std::array<int, 2> const axles{
        world.addRevoluteJoint(chassis, rearWheel, {-axleOffset, 34.f}, {}),
        world.addRevoluteJoint(chassis, frontWheel, {axleOffset, 34.f}, {})};
    for (int axle : axles) {
        auto& motor = static_cast<phys2d::Revolute&>(world.joint(axle));
        motor.enableMotor = true;
        motor.motorSpeed = 8.f;
        motor.maxMotorTorque = 1200.f;
    }
    return world;
}

} // namespace

int main() {
    InitWindow(visual::screenWidth, visual::screenHeight, "phys2d - powered vehicle");
    SetTargetFPS(60);

    phys2d::World world = makeWorld();
    float accumulator = 0.f;
    bool paused = false;
    float motorSpeed = 8.f;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            world = makeWorld();
            accumulator = 0.f;
            paused = false;
            motorSpeed = 8.f;
        }
        if (IsKeyDown(KEY_RIGHT)) {
            motorSpeed = std::min(motorSpeed + 12.f * GetFrameTime(), 16.f);
        }
        if (IsKeyDown(KEY_LEFT)) {
            motorSpeed = std::max(motorSpeed - 12.f * GetFrameTime(), 0.f);
        }
        for (std::size_t i = 0; i < world.jointCount(); ++i) {
            auto& motor = static_cast<phys2d::Revolute&>(world.joint(static_cast<int>(i)));
            motor.motorSpeed = motorSpeed;
        }
        visual::advance(world, accumulator, paused);

        phys2d::Body const& car = world.body(terrainSegments);
        phys2d::Body const& rearWheel = world.body(terrainSegments + 1);
        phys2d::Body const& frontWheel = world.body(terrainSegments + 2);
        Camera2D const view = visual::camera(car.pos, 1.f);
        BeginDrawing();
        ClearBackground(Color{235, 241, 248, 255});
        BeginMode2D(view);
        drawShock(car.worldPoint({-28.f, 12.f}), rearWheel.pos + phys2d::Vec2{0.f, -12.f});
        drawShock(car.worldPoint({28.f, 12.f}), frontWheel.pos + phys2d::Vec2{0.f, -12.f});
        EndMode2D();
        visual::drawWorld(world, view);
        visual::drawHud("Vehiculo con suspension visible",
                        "Flechas: acelera/frena   Espacio: pausa   R: reinicia", paused);
        DrawText(TextFormat("motor: %.1f rad/s   x: %.0f", motorSpeed, car.pos.x), 34, 108, 18,
                 Color{50, 64, 83, 255});
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
