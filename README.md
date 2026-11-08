# phys2d — API de físicas 2D C++23 (header-only)

Ultra fácil: `#include "phys2d/phys2d.hpp"` y listo. Sin dependencias.

Soy muy flojo, así que hice esto para no volver a pelearme con las matemáticas. Tampoco hay que ser muy listo para usarlo, como yo. La librería está hecha mayormente en inglés, pero el tutorial está en español. En el futuro será en inglés. Disfrútalo.

Documentación extensa: [docs/](docs/README.md) — `core`, `collision`, `joints`, `queries`.

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

auto hit = world.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f); // hit.hit, hit.point
phys2d::Vec2 pos = world.body(ball).pos;
for (auto& body : world) { /* itera todos los cuerpos */ }
```

Qué incluye:

- Formas: círculo, caja (`AABB` + `Body::angle`), polígono convexo, cápsula y
  segmento (`std::variant`), con inicializadores designados.
- Solver secuencial de impulsos **con torque**: masa efectiva por punto de
  contacto (hasta 2 puntos por manifold), fricción, restitución, warm
  starting y corrección posicional lineal + angular.
- Broadphase SpatialHash o SAP (`world.broadPhaseType`), substeps,
  `bullet` (multiplica substeps), sleep opcional, eventos
  `onBeginContact`/`onEndContact`.
- Joints integrados en `World::step`: `Distance`, `Revolute`, `Prismatic`,
  `Pulley`, `Spring`, `Weld`, `Gear`, `Rope`, `Slider`
  (`world.addJoint<phys2d::Distance>(a, b, rest)`, o helpers
  `world.addRevoluteJoint(a, b, {0, 0}, {0, 0})` que aceptan `{0, 0}` a pelo).
- CCD en dos capas: (1) swept con time-of-impact — círculo exacto contra
  cualquier círculo/AABB/cápsula/segmento; convexos contra convexos exacto por
  diferencia de Minkowski (`shapeCastShape`/`shapeCastBodyShape`); el resto
  con `shapeCastBody` (barrido del centro, con radio cuando aplica) — una
  bala a 500.000 px/s se detiene en la pared, y el cuerpo conserva el
  movimiento restante del substep tras el impacto; (2) substeps adaptativos
  como red de seguridad (si un cuerpo se mueve más de medio radio por step, se
  subdivide, tope 128).
- Persistencia de manifolds: pares cuyos cuerpos no se movieron
  (estáticos/dormidos) reutilizan el manifold sin re-colisionar.
- Sleep activado por defecto: los cuerpos quietos duermen
  (`isSleeping`, `invMass=0`); despiertan con `applyForce`, con un
  contacto cuyo compañero **se mueve**, o con un joint cuyo compañero se
  mueve.
- Callbacks `onBeginContact`/`onEndContact`: una notificación **por step**
  (no por substep), y los pares estático-estático no los disparan.
- `DebugDraw` completo: `drawPolygon`/`drawCapsule` con implementación por
  defecto (tus `DebugDraw` antiguos con solo 3 métodos siguen compilando).
- Queries: `world.raycast(...)`, `raycastPoint`/`raycastCircle`/`raycastAABB`/
  `raycastPolygon`/`raycastCapsule`, `shapeCastCircle`/`shapeCastBody`/
  `shapeCastPoint` (barrido de un círculo), `pointInShape`,
  `closestPointOnShape`, `overlapAABB`, `DebugDraw` + `drawWorld`.
- Copiar un `World` clona cuerpos **y** joints (`Joint::clone()`).
- `World::bodiesSpan()` devuelve `std::span<const Body*>` (solo lectura);
  para mutar usa `world.body(id)` o la iteración (`for (auto& b : world)`).
- Joints por índice: `world.jointCount()` y `world.joint(id)` (lanza
  `std::out_of_range`), típicamente para ajustar parámetros:
  `static_cast<phys2d::Revolute&>(world.joint(j)).motorSpeed = 2.f`.
  `addJoint` valida `a`/`b`: lanza `std::out_of_range` si están fuera de
  rango y `std::invalid_argument` si `a == b`.
- `world.contacts()` devuelve `std::span<const Manifold>` (solo lectura).
- Errores explícitos: `world.body(id)` lanza `std::out_of_range` si el id no
  existe; `step(dt)` con `dt` no finito o ≤ 0 lanza `std::invalid_argument`;
  crear una forma degenerada (radio ≤ 0, polígono de <3 vértices o no
  convexo, masa dinámica ≤ 0, fricción < 0, restitución fuera de [0, 1],
  amortiguamiento < 0, segmento con `radius < 0`, cápsula con `dir` nulo o
  NaN, vértices de polígono no finitos) o con valores no finitos
  (NaN/inf en pos, vel, ángulo…) lanza `std::invalid_argument`.

## Cambios rompientes (API v3)

- `World::bodies()` y `bodySpan()` eliminados → `bodiesSpan()` (span de
  punteros const). Los cuerpos se mutan vía `world.body(id)`/iteración.
- `World::joints()` eliminado → `jointCount()` + `joint(id)` (devuelve
  `Joint&` y lanza `std::out_of_range`).
- `World::contacts()` no-const devuelve `std::span<const Manifold>`.
- `step(dt)` lanza con `dt ≤ 0` o NaN/inf; ya no trunca `dt > 0.05` en
  silencio (los dt grandes se subdividen en substeps adaptativos).
- `addJoint` valida ids (fuera de rango o `a == b`).
- `Revolute::referenceAngle` eliminado: el motor es una restricción de
  velocidad pura (`ωB − ωA = motorSpeed` en rad/s). Para fijar el ángulo
  relativo usa `Weld`.
- `Revolute`/`Distance`/…: fórmulas del muelle e impulso de amortiguación
  corregidas y documentadas en [docs/joints.md](docs/joints.md).
- Segmentos con `radius > 0` son cápsulas finas de verdad en **toda** la
  narrowphase (antes el grosor solo afectaba a masa y AABB).
- `onBeginContact`/`onEndContact` disparan una vez por step, no por
  substep, y los pares estático-estático no disparan callbacks.
- `Manifold::count` vale `0` por defecto (antes `2`); un `collide()`
  fallido deja la manifold intacta salvo lo que fije el llamante.
- Materia de CCD: círculos barrean contra **cualquier** forma; el punto de
  impacto usa `closestPointOnShape` (no el casco de Minkowski) y la parte
  del substep posterior al impacto se conserva.
- `Prismatic::axis` ya no se normaliza en `prepare()` (se usa `axisNorm`
  interno); `Prismatic::lower/upper` sin cambios.

## Compilar y probar

```bash
make        # demo + ejemplos + tests_bin
make test   # tests con asserts + ejemplos
make bench  # benchmark: ./bench [n] [steps]  (defecto 10000 cuerpos x 120 steps)
make sanitizers  # tests con ASan+UBSan
make clean
```

Rendimiento medido en esta máquina (círculos en pila, broadphase con radix
sort, radios cacheados, buffers reutilizables y sleep operativo): ~0.35 ms/step
con 500 cuerpos y ~1.9 ms/step con 2.000 (bench `./bench <n> <steps>`).
El coste crece con el nº de contactos, no solo con el nº de cuerpos.

Flags: `g++ -std=c++23 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror`.
Formato: `.clang-format` (K&R, 4 espacios, 100 columnas) — `clang-format --dry-run --Werror` limpio.

## Licencia

MIT — ver [LICENSE](LICENSE).
