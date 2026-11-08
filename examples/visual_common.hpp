#pragma once

#include "phys2d/phys2d.hpp"
#include <algorithm>
#include <array>
#include <raylib.h>
#include <span>
#include <vector>

namespace visual {

inline constexpr int screenWidth = 1280;
inline constexpr int screenHeight = 720;
inline constexpr float fixedStep = 1.f / 120.f;

[[nodiscard]] inline Vector2 toRaylib(phys2d::Vec2 point) noexcept {
    return {point.x, point.y};
}

[[nodiscard]] inline phys2d::Vec2 fromRaylib(Vector2 point) noexcept {
    return {point.x, point.y};
}

struct RaylibDraw final : phys2d::DebugDraw {
    Color color{230, 235, 245, 255};
    Color outline{25, 32, 45, 255};

    void drawCircle(phys2d::Vec2 center, float radius) override {
        DrawCircleV(toRaylib(center), radius, color);
        DrawCircleLinesV(toRaylib(center), radius, outline);
    }

    void drawBox(phys2d::Vec2 center, phys2d::Vec2 half, float angle) override {
        Rectangle const rect{center.x, center.y, half.x * 2.f, half.y * 2.f};
        Vector2 const origin{half.x, half.y};
        DrawRectanglePro(rect, origin, angle * RAD2DEG, color);
        std::array<phys2d::Vec2, 4> corners{phys2d::Vec2{-half.x, -half.y}.rotated(angle) + center,
                                            phys2d::Vec2{half.x, -half.y}.rotated(angle) + center,
                                            phys2d::Vec2{half.x, half.y}.rotated(angle) + center,
                                            phys2d::Vec2{-half.x, half.y}.rotated(angle) + center};
        for (std::size_t i = 0; i < corners.size(); ++i) {
            DrawLineEx(toRaylib(corners[i]), toRaylib(corners[(i + 1U) % corners.size()]), 2.f,
                       outline);
        }
    }

    void drawSegment(phys2d::Vec2 a, phys2d::Vec2 b) override {
        DrawLineEx(toRaylib(a), toRaylib(b), 3.f, color);
    }

    void drawPolygon(std::span<phys2d::Vec2 const> vertices) override {
        if (vertices.size() < 3U) {
            return;
        }
        std::vector<Vector2> points;
        points.reserve(vertices.size());
        for (phys2d::Vec2 vertex : vertices) {
            points.push_back(toRaylib(vertex));
        }
        DrawTriangleFan(points.data(), static_cast<int>(points.size()), color);
        for (std::size_t i = 0; i < vertices.size(); ++i) {
            DrawLineEx(points[i], points[(i + 1U) % points.size()], 2.f, outline);
        }
    }

    void drawCapsule(phys2d::Vec2 a, phys2d::Vec2 b, float radius) override {
        DrawLineEx(toRaylib(a), toRaylib(b), radius * 2.f, color);
        DrawCircleV(toRaylib(a), radius, color);
        DrawCircleV(toRaylib(b), radius, color);
        DrawCircleLinesV(toRaylib(a), radius, outline);
        DrawCircleLinesV(toRaylib(b), radius, outline);
    }

    void drawThickSegment(phys2d::Vec2 a, phys2d::Vec2 b, float radius) {
        drawCapsule(a, b, radius);
    }
};

inline void drawBodies(phys2d::World const& world) {
    RaylibDraw draw{};
    for (phys2d::Body const* body : world.bodiesSpan()) {
        if (body == nullptr) {
            continue;
        }
        if (body->isStatic()) {
            draw.color = Color{83, 101, 125, 255};
        } else if (body->bullet) {
            draw.color = Color{255, 117, 76, 255};
        } else if (body->isSleeping) {
            draw.color = Color{111, 130, 153, 255};
        } else {
            draw.color = Color{89, 190, 217, 255};
        }
        if (auto const* segment = std::get_if<phys2d::Segment>(&body->shape);
            segment != nullptr && segment->radius > 0.f) {
            draw.drawThickSegment(body->worldPoint(segment->a), body->worldPoint(segment->b),
                                  segment->radius);
        } else {
            (void)phys2d::drawBody(draw, *body);
        }
    }
}

inline void drawContacts(phys2d::World const& world) {
    for (phys2d::Manifold const& contact : world.contacts()) {
        if (contact.count > 0) {
            DrawCircleV(toRaylib(contact.point), 4.f, Color{255, 220, 95, 255});
        }
        if (contact.count > 1) {
            DrawCircleV(toRaylib(contact.point2), 4.f, Color{255, 220, 95, 255});
        }
    }
}

inline void drawHud(char const* title, char const* controls, bool paused) {
    DrawRectangle(18, 18, 760, 78, Color{16, 23, 36, 220});
    DrawText(title, 34, 30, 25, RAYWHITE);
    DrawText(controls, 34, 64, 16, Color{190, 203, 220, 255});
    if (paused) {
        DrawText("PAUSA", screenWidth - 120, 28, 22, Color{255, 220, 95, 255});
    }
}

inline void advance(phys2d::World& world, float& accumulator, bool& paused) {
    if (IsKeyPressed(KEY_SPACE)) {
        paused = !paused;
    }
    if (paused) {
        return;
    }
    accumulator += std::min(GetFrameTime(), 0.05f);
    while (accumulator >= fixedStep) {
        world.step(fixedStep);
        accumulator -= fixedStep;
    }
}

[[nodiscard]] inline Camera2D camera(phys2d::Vec2 target, float zoom = 1.f) noexcept {
    return Camera2D{
        .offset = {static_cast<float>(screenWidth) * 0.5f, static_cast<float>(screenHeight) * 0.5f},
        .target = toRaylib(target),
        .rotation = 0.f,
        .zoom = zoom};
}

inline void drawWorld(phys2d::World const& world, Camera2D const& view) {
    BeginMode2D(view);
    drawBodies(world);
    drawContacts(world);
    EndMode2D();
}

} // namespace visual
