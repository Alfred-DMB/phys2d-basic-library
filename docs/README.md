# phys2d — Documentación

Motor de físicas 2D en C++23, header-only, sin dependencias.

Índice:

| Doc | Contenido |
|---|---|
| [core.md](core.md) | `Vec2`, `Mat2`, formas, `Body`, `World` (`add*`, `step`, broadphase, sleep, callbacks) |
| [collision.md](collision.md) | `Manifold`, `collide`, los 15 pares, SAT+clip, solver con torque, fricción/restitución, CCD swept, `shapeCast`, persistencia |
| [joints.md](joints.md) | `Distance`, `Revolute`, `Prismatic`, `Pulley`, `Spring`, `Weld`, `Gear`, `Rope`, `Slider`; helpers `addJoint`; clonar `World` |
| [queries.md](queries.md) | `raycast*`, `shapeCast*`, `pointInShape`, `closestPointOnShape`, `overlapAABB`, `DebugDraw`/`drawWorld`, bench |

Empezar:

```cpp
#include "phys2d/phys2d.hpp"
phys2d::World world;
world.gravity = {0.f, 900.f};
int floor = world.addStaticBox(
    phys2d::BoxParams{.pos = {400.f, 590.f}, .half = {500.f, 10.f}});
int ball = world.addDynamicCircle(
    phys2d::CircleParams{.pos = {100.f, 50.f}, .radius = 20.f, .vel = {40.f, 0.f}});
world.addJoint<phys2d::Distance>(floor, ball, 150.f); // opcional
for (int i = 0; i < 120; ++i) world.step(1.f / 60.f);
auto hit = world.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f);
```

Verificar cambios: `make test`, `make sanitizers`, `make bench`.

Ejemplos ejecutables: `demo`, `ex_stack`, `ex_chain`, `ex_raycast` y
`ex_vehicle` (dos ruedas motrices conectadas al chasis con joints `Revolute`).

Los demos gráficos opcionales usan raylib: `make visual` compila
`visual_vehicle`, `visual_stack` y `visual_bullet`. Controles e instalación en
[README.md](../README.md); no se requieren para `make test`.
