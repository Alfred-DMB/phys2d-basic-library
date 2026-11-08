#include "phys2d/phys2d.hpp"

#include <charconv>
#include <chrono>
#include <print>
#include <system_error>

namespace {

[[nodiscard]] int parsePosInt(char const* arg, int fallback) noexcept {
    if (arg == nullptr) {
        return fallback;
    }
    int value = 0;
    char const* const begin = arg;
    char const* const end = arg + std::char_traits<char>::length(arg);
    auto const res = std::from_chars(begin, end, value, 10);
    if (res.ec != std::errc() || res.ptr != end || value <= 0) {
        return fallback;
    }
    return value;
}

} // namespace

int main(int argc, char* argv[]) {
    constexpr int kDefaultN = 10000;
    constexpr int kDefaultSteps = 120;
    constexpr int kCols = 100;

    constexpr float kDefaultCellSize = 64.f;
    int n = kDefaultN;
    int steps = kDefaultSteps;
    float cellSize = kDefaultCellSize;
    phys2d::World::BroadPhaseType broadphase = phys2d::World::BroadPhaseType::SpatialHash;
    if (argc > 1) {
        n = parsePosInt(argv[1], kDefaultN);
    }
    if (argc > 2) {
        steps = parsePosInt(argv[2], kDefaultSteps);
    }
    if (argc > 3) {
        cellSize = static_cast<float>(parsePosInt(argv[3], static_cast<int>(kDefaultCellSize)));
    }
    if (argc > 4) {
        std::string_view bpArg = argv[4];
        if (bpArg == "sap") {
            broadphase = phys2d::World::BroadPhaseType::SAP;
        } else {
            broadphase = phys2d::World::BroadPhaseType::SpatialHash;
        }
    }

    phys2d::World world;
    world.gravity = phys2d::Vec2{0.f, 900.f};
    world.cellSize = cellSize;
    world.broadPhaseType = broadphase;
    int const floorId = world.addStaticBox(phys2d::Vec2{1100.f, 590.f}, phys2d::Vec2{2000.f, 10.f});
    (void)floorId;

    for (int i = 0; i < n; ++i) {
        int const col = i % kCols;
        int const row = i / kCols;
        float const x = 100.f + static_cast<float>(col) * 20.f;
        float const y = 50.f + static_cast<float>(row) * 20.f;
        int const id = world.addDynamicCircle(phys2d::Vec2{x, y}, 8.f);
        (void)id;
    }

    auto const t0 = std::chrono::steady_clock::now();
    for (int s = 0; s < steps; ++s) {
        world.step(1.f / 60.f);
    }
    auto const t1 = std::chrono::steady_clock::now();

    double const totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    double perStepMs = 0.0;
    if (steps > 0) {
        perStepMs = totalMs / static_cast<double>(steps);
    }

    std::println("bench N={} steps={} cellSize={:.0f} broadphase={} total={:.2f}ms "
                 "per-step={:.3f}ms contacts={} bodies={}",
                 n, steps, cellSize,
                 broadphase == phys2d::World::BroadPhaseType::SAP ? "sap" : "sh", totalMs,
                 perStepMs, world.contacts().size(), world.size());
    return 0;
}
