#include "visual_common.hpp"

namespace {

phys2d::World makeWorld() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};
    (void)world.addStaticBox(
        phys2d::BoxParams{.pos = {640.f, 690.f}, .half = {1800.f, 20.f}, .friction = 0.9f});

    constexpr float boxSize = 34.f;
    for (int row = 0; row < 7; ++row) {
        for (int column = 0; column < 5 - row / 2; ++column) {
            float const x = 520.f + static_cast<float>(column) * (boxSize * 2.f + 2.f) +
                            static_cast<float>(row % 2) * boxSize;
            float const y = 650.f - boxSize - static_cast<float>(row) * (boxSize * 2.f);
            (void)world.addDynamicBox(
                phys2d::BoxParams{.pos = {x, y}, .half = {boxSize, boxSize}, .friction = 0.75f});
        }
    }
    return world;
}

} // namespace

int main() {
    InitWindow(visual::screenWidth, visual::screenHeight, "phys2d - interactive stack");
    SetTargetFPS(60);

    phys2d::World world = makeWorld();
    Camera2D const view = visual::camera({640.f, 360.f});
    float accumulator = 0.f;
    bool paused = false;
    Vector2 previousMouse = GetMousePosition();

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            world = makeWorld();
            accumulator = 0.f;
            paused = false;
        }
        visual::advance(world, accumulator, paused);

        phys2d::Vec2 const mouse = visual::fromRaylib(GetScreenToWorld2D(GetMousePosition(), view));
        phys2d::Vec2 const launchVelocity{
            std::clamp((mouse.x - previousMouse.x) * 8.f, -900.f, 900.f),
            std::clamp((mouse.y - previousMouse.y) * 8.f, -900.f, 900.f)};
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            (void)world.addDynamicCircle(phys2d::CircleParams{
                .pos = mouse, .radius = 18.f, .vel = launchVelocity, .friction = 0.75f});
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            (void)world.addDynamicBox(phys2d::BoxParams{
                .pos = mouse, .half = {22.f, 22.f}, .vel = launchVelocity, .angle = 0.25f});
        }
        previousMouse = {mouse.x, mouse.y};

        BeginDrawing();
        ClearBackground(Color{235, 241, 248, 255});
        visual::drawWorld(world, view);
        visual::drawHud("Pila interactiva",
                        "Click izq: circulo   Click der: caja   Espacio: pausa   R: reinicia",
                        paused);
        DrawText(TextFormat("cuerpos: %i", static_cast<int>(world.size())), 34, 108, 18,
                 Color{50, 64, 83, 255});
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
