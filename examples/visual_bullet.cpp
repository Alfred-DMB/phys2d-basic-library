#include "visual_common.hpp"

namespace {

constexpr float wallX = 900.f;
constexpr float sweptLaneY = 310.f;
constexpr float discreteLaneY = 510.f;
constexpr float wallHalfThickness = 1.f;
constexpr float projectileHalfWidth = 2.f;
constexpr float projectileHalfLength = 55.f;
constexpr float initialSpeed = 1200.f;

struct Shot {
    int swept = -1;
    int discrete = -1;
};

phys2d::World makeWorld() {
    phys2d::World world;
    world.gravity = {};
    world.positionIterations = 5;
    for (float laneY : {sweptLaneY, discreteLaneY}) {
        (void)world.addStaticBox(phys2d::BoxParams{.pos = {wallX, laneY},
                                                   .half = {wallHalfThickness, 85.f},
                                                   .friction = 0.4f,
                                                   .restitution = 0.f});
    }
    return world;
}

int addProjectile(phys2d::World& world, float y, float speed, bool enableBullet) {
    int const id =
        world.addDynamicBox(phys2d::BoxParams{.pos = {180.f, y},
                                              .half = {projectileHalfWidth, projectileHalfLength},
                                              .vel = {speed, 0.f},
                                              .mass = 2.f,
                                              .friction = 0.2f,
                                              .restitution = 0.f,
                                              .linearDamping = 0.f,
                                              .angularDamping = 0.f});
    phys2d::Body& projectile = world.body(id);
    projectile.fixedRotation = true;
    projectile.updateInertia();
    projectile.bullet = enableBullet;
    return id;
}

Shot fire(phys2d::World& world, float speed) {
    return {addProjectile(world, sweptLaneY, speed, true),
            addProjectile(world, discreteLaneY, speed, false)};
}

} // namespace

int main() {
    InitWindow(visual::screenWidth, visual::screenHeight, "phys2d - CCD comparison");
    SetTargetFPS(60);

    phys2d::World world = makeWorld();
    Shot shot{};
    float speed = initialSpeed;
    float accumulator = 0.f;
    bool paused = false;
    bool fired = false;
    bool sweptHit = false;
    bool discretePassed = false;
    bool resultRecorded = false;
    int completedTrials = 0;
    int stoppedWithCcd = 0;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            world = makeWorld();
            shot = {};
            accumulator = 0.f;
            paused = false;
            fired = false;
            sweptHit = false;
            discretePassed = false;
            resultRecorded = false;
            completedTrials = 0;
            stoppedWithCcd = 0;
        }
        if (IsKeyDown(KEY_UP)) {
            speed = std::min(speed + 1800.f * GetFrameTime(), 3600.f);
        }
        if (IsKeyDown(KEY_DOWN)) {
            speed = std::max(speed - 1800.f * GetFrameTime(), 300.f);
        }
        if (IsKeyPressed(KEY_F) && (!fired || resultRecorded)) {
            if (resultRecorded) {
                world = makeWorld();
            }
            shot = fire(world, speed);
            fired = true;
            sweptHit = false;
            discretePassed = false;
            resultRecorded = false;
            accumulator = 0.f;
        }
        visual::advance(world, accumulator, paused);

        if (fired) {
            phys2d::Body const& swept = world.body(shot.swept);
            phys2d::Body const& discrete = world.body(shot.discrete);
            sweptHit = sweptHit || swept.pos.x >= wallX - wallHalfThickness - projectileHalfWidth;
            discretePassed =
                discretePassed || discrete.pos.x > wallX + wallHalfThickness + projectileHalfWidth;
            if (sweptHit && discretePassed && !resultRecorded) {
                ++completedTrials;
                ++stoppedWithCcd;
                resultRecorded = true;
            }
        }

        BeginDrawing();
        ClearBackground(Color{235, 241, 248, 255});
        visual::drawWorld(
            world,
            Camera2D{.offset = {0.f, 0.f}, .target = {0.f, 0.f}, .rotation = 0.f, .zoom = 1.f});
        DrawLineEx({130.f, sweptLaneY}, {1130.f, sweptLaneY}, 1.f, Color{190, 201, 214, 255});
        DrawLineEx({130.f, discreteLaneY}, {1130.f, discreteLaneY}, 1.f, Color{190, 201, 214, 255});
        visual::drawHud("CCD: comparacion lado a lado",
                        "F: dispara pareja   Flechas: velocidad   Espacio: pausa   R: reinicia",
                        paused);
        DrawText("bullet ON  -  barrido continuo", 160, 205, 20, Color{34, 133, 87, 255});
        DrawText("bullet OFF  -  colision discreta", 160, 405, 20, Color{190, 88, 55, 255});
        DrawText(TextFormat("velocidad: %.0f px/s", speed), 960, 205, 19, Color{50, 64, 83, 255});
        DrawText(sweptHit ? "DETENIDA en la pared fina" : (fired ? "en camino..." : "pulsa F"), 160,
                 230, 17, Color{50, 64, 83, 255});
        DrawText(discretePassed ? "ATRAVESO la pared" : (fired ? "en camino..." : "pulsa F"), 160,
                 430, 17, Color{50, 64, 83, 255});
        DrawText(TextFormat("pruebas: %i   detenidas con CCD: %i", completedTrials, stoppedWithCcd),
                 960, 230, 17, Color{50, 64, 83, 255});
        DrawText("Proyectil largo y estrecho; pared de 2 px.", 160, 640, 17,
                 Color{80, 94, 112, 255});
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
