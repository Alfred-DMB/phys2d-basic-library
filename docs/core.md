# phys2d — Núcleo (Vec2, Mat2, shapes, Body, World)

> Header-only, C++23, sin dependencias. `#include "phys2d/phys2d.hpp"` es la
> única inclusión necesaria. Coordenadas en **píxeles**, `float` (32 bits),
> eje **Y hacia abajo** (pantalla), radianes para ángulos, `gravity = {0, 900}`.
> Todas las formas viven en **espacio local** respecto a `Body::pos`/`Body::angle`;
> la rotación de la caja es `Body::angle`, no se guarda dentro de la forma.

---

## 1. Vec2 — `include/phys2d/vec2.hpp` (136 líneas)

Vector 2D trivial, en píxeles, `constexpr`/`noexcept` en la práctica totalidad
de su API. No hace asignaciones.

### 1.1 Datos

```cpp
struct Vec2 {
    float x = 0.f;
    float y = 0.f;
    constexpr Vec2() noexcept;
    constexpr Vec2(float x_, float y_) noexcept;
};
```

- `Vec2{}` ⇒ `(0, 0)`. Es seguro usarlo como valor por defecto / "nulo".
- Se construye con dos flotantes, `Vec2{3.f, 4.f}`, o en parámetros como
  `{400.f, 590.f}` cuando el tipo destino ya es `Vec2` (conversión por
  lista). Ojo: en inicializadores designados de params **no** se puede mezclar
  `{{100.f, 50.f}, .radius = 20.f}`; usa todo-posicional o todo-designado
  (ver §3.5).

### 1.2 Operadores

| Operación | Significado |
|---|---|
| `a + b`, `a - b`, `-a` | suma / resta / negación |
| `a * s`, `s * a` | escala |
| `a / s` | división escalar (multiplica por el recíproco implícito) |
| `a += b`, `a -= b`, `a *= s`, `a /= s` | mutantes, devuelven `*this` |
| `a == b` | comparación exacta campo a campo (`= default`) |

```cpp
phys2d::Vec2 a{3.f, 4.f};
phys2d::Vec2 b = a * 2.f;   // (6, 8)
phys2d::Vec2 c = 2.f * a;   // (6, 8) — escalar por la izquierda
b += a;                     // (9, 12)
```

> **Comparación exacta**: `Vec2::operator==` es igualdad bit a bit por campo.
> Tras aritmética en coma flotante, dos vectores que "deberían" ser iguales
> casi nunca lo son. Para distancias/`atan2` usa `length2()` y compara contra
> un épsilon (p. ej. `1e-8f`), como hace la propia biblioteca.

### 1.3 Producto escalar y producto cruzado

```cpp
[[nodiscard]] constexpr float dot(Vec2 other) const noexcept;   // x·x' + y·y'
[[nodiscard]] constexpr float cross(Vec2 other) const noexcept; // x·y' − y·x'  (escalar z)
```

- `dot` es el producto escalar clásico: `|a|·|b|·cos(θ)`.
- `cross` en 2D devuelve **un escalar** (la componente z del producto
  tridimensional). Positivo si `other` está en sentido antihorario desde `a`
  en un sistema de ejes usual (sin tener en cuenta que Y crece hacia abajo).

```cpp
phys2d::Vec2 a{1.f, 0.f}, b{0.f, 1.f};
float d = a.dot(b);   // 0
float c = a.cross(b); // 1
```

Usos típicos: proyectar sobre una normal (`dot`), áreas signadas,
perpuntear, y el torque (`r.cross(f)`).

### 1.4 Longitud y normalización

```cpp
[[nodiscard]] constexpr float length2() const noexcept;   // |v|² = dot(v,v)
[[nodiscard]] constexpr float lengthSq() const noexcept;  // alias de length2
[[nodiscard]] float length() const noexcept;              // sqrt(length2())
[[nodiscard]] Vec2 normalized() const noexcept;           // v / |v|, o (0,0) si |v| ≤ 1e-8
[[nodiscard]] Vec2 normalizedOr(Vec2 fallback) const noexcept; // v/|v|, o fallback
```

- `normalized()` nunca produce NaN/inf: un vector nulo (o de magnitud
  ≤ `1e-8f`) devuelve `Vec2{}`.
- `normalizedOr(fallback)` devuelve `fallback` cuando el vector es degenerado;
  útil para normales de impacto ante velocidades casi nulas.
- Regla de estilo: compara siempre con `length2()` antes que con `length()`
  (evita `sqrt`). La propia API compara contra `1e-8f` / `1e-12f`.

### 1.5 Perpendicular, rotación, reflexión, interpolación, clamp

```cpp
[[nodiscard]] constexpr Vec2 perpendicular() const noexcept; // {-y, x} — 90° antihorario
[[nodiscard]] Vec2 rotated(float angle) const noexcept;      // rotación por ángulo (rad)
[[nodiscard]] constexpr Vec2 reflect(Vec2 normal) const noexcept; // v − n·(2·v·n)
[[nodiscard]] constexpr Vec2 lerp(Vec2 other, float t) const noexcept;
[[nodiscard]] constexpr Vec2 clamp(Vec2 lo, Vec2 hi) const noexcept; // clamp por componente
[[nodiscard]] float angle() const noexcept;                  // atan2(y, x) en (−π, π]
```

- `perpendicular()` devuelve `(-y, x)`, un vector rotado +90° en el sistema
  de ejes matemático. Con Y hacia abajo en pantalla, visualmente es −90°,
  pero la convención algebraica es la que usa el solver (torque, áreas).
- `rotated(angle)` usa `cos`/`sin` por llamada: **no** es `constexpr` y es la
  operación dominante en coste en bucles. En hot paths precalcula `Mat2::rotation`.
- `reflect(normal)` asume `normal` normalizado; si no lo está, el resultado
  no es una reflexión geométrica.
- `lerp(other, t)`: `t=0` ⇒ `*this`, `t=1` ⇒ `other`. No clampéa `t`.
- `clamp(lo, hi)` clampéa cada componente de forma independiente (rectángulo,
  no círculo).
- `angle()` devuelve el ángulo polar en radianes.

### 1.6 Funciones libres (fuera de la clase)

```cpp
[[nodiscard]] constexpr Vec2 operator*(float s, Vec2 v) noexcept;
[[nodiscard]] constexpr Vec2 min(Vec2 a, Vec2 b) noexcept;   // componente a componente
[[nodiscard]] constexpr Vec2 max(Vec2 a, Vec2 b) noexcept;
[[nodiscard]] constexpr Vec2 lerp(Vec2 a, Vec2 b, float t) noexcept;
[[nodiscard]] constexpr Vec2 clamp(Vec2 v, Vec2 lo, Vec2 hi) noexcept;
[[nodiscard]] constexpr Vec2 abs(Vec2 v) noexcept;
[[nodiscard]] float distance(Vec2 a, Vec2 b) noexcept;
[[nodiscard]] float distance2(Vec2 a, Vec2 b) noexcept;
std::ostream& operator<<(std::ostream& os, Vec2 v);           // "(x, y)"
```

`min`/`max` sobre `Vec2` son **componente a componente**, no "el de menor
longitud". `abs` es por componente. `operator<<` imprime `(x, y)` sin newline.
> `std::println("{}")` con un `Vec2` **no compila**: no hay especialización de
> `std::formatter` para `Vec2` (solo `operator<<` para flujos). Imprime los
> componentes sueltos: `std::println("({:.1f},{:.1f})", v.x, v.y)`.

### 1.7 Ejemplo compilable

```cpp
#include "phys2d/vec2.hpp"
#include <print>

int main() {
    using namespace phys2d;
    Vec2 a{3.f, 4.f}, b{1.f, 0.f};

    std::println("a = ({:.2f},{:.2f})  |a|={:.2f}  angle={:.3f}", a.x, a.y, a.length(), a.angle());
    Vec2 an = a.normalized(); std::println("a.normalized() = ({:.3f},{:.3f})", an.x, an.y);
    Vec2 ap = a.perpendicular(); std::println("a.perpendicular() = ({:.1f},{:.1f})", ap.x, ap.y);
    Vec2 ar = a.rotated(1.5707963f); std::println("a.rotated(pi/2) = ({:.1f},{:.3f})", ar.x, ar.y);
    Vec2 rf = a.reflect(b); std::println("a.reflect(b) = ({:.1f},{:.1f})", rf.x, rf.y); // (-3, 4)
    std::println("dist(a, b) = {:.2f}", distance(a, b));
    Vec2 mn = min(a, b); std::println("min(a,b) = ({:.1f},{:.1f})", mn.x, mn.y);
    return 0;
}
```

---

## 2. Mat2 — `include/phys2d/mat2.hpp` (85 líneas)

Matriz 2×2 en **orden de columnas** (column-major): `M * v = col0 * v.x + col1 * v.y`.

```cpp
struct Mat2 {
    Vec2 col0{1.f, 0.f};
    Vec2 col1{0.f, 1.f};
};
```

- Sin columnas es identidad.
- El constructor de 4 flotantes es **`Mat2(m00, m01, m10, m11)` en orden de
  filas lógico**, que internamente mapea a `(col0={m00,m10}, col1={m01,m11})`.
  No es ambiguo si lo piensas como "escribo filas, se almacena por columnas".

### 2.1 Accesores y operadores

```cpp
[[nodiscard]] constexpr Vec2 row0() const noexcept;  // {col0.x, col1.x}
[[nodiscard]] constexpr Vec2 row1() const noexcept;  // {col0.y, col1.y}
[[nodiscard]] constexpr Vec2 operator*(Vec2 v) const noexcept;  // col0·v.x + col1·v.y
[[nodiscard]] constexpr Mat2 operator*(Mat2 other) const noexcept;
[[nodiscard]] constexpr Mat2 operator*(float s) const noexcept;
constexpr Mat2& operator*=(float s) noexcept;
constexpr Mat2& operator*=(Mat2 other) noexcept;
[[nodiscard]] constexpr bool operator==(Mat2 const& other) const noexcept = default;
```

`operator*(Mat2)` multiplica `(*this) * other.col0` y `(*this) * other.col1`:
composición estándar, no conmutativa.

### 2.2 Constructores estáticos

```cpp
[[nodiscard]] static constexpr Mat2 identity() noexcept;
[[nodiscard]] static constexpr Mat2 zero() noexcept;
[[nodiscard]] static Mat2 rotation(float angle) noexcept;      // {{c, s}, {-s, c}}
[[nodiscard]] static Mat2 fromAngle(float angle) noexcept;     // alias de rotation
[[nodiscard]] static constexpr Mat2 scaling(Vec2 scale) noexcept;
[[nodiscard]] static constexpr Mat2 fromColumns(Vec2 c0_, Vec2 c1_) noexcept;
[[nodiscard]] static constexpr Mat2 fromRows(Vec2 r0_, Vec2 r1_) noexcept;
```

- `rotation(angle)` = `{{c, s}, {-s, c}}`, es decir columnas `(c, s)` y
  `(-s, c)`. Llamemos `rot(a)`: entonces `rot(a) * v == v.rotated(a)`.
- `fromRows(r0, r1)` construye columnas `{r0.x, r1.x}` / `{r0.y, r1.y}`.
- `zero()` no es identidad: es todo ceros.

### 2.3 Determinante, traspuesta, inversa

```cpp
[[nodiscard]] constexpr float determinant() const noexcept;   // col0.x·col1.y − col0.y·col1.x
[[nodiscard]] constexpr Mat2 transpose() const noexcept;
[[nodiscard]] constexpr Mat2 inverse() const noexcept;
```

- `inverse()` devuelve la matriz **cero** si `determinant() == 0` (no lanza
  ni devuelve NaN). Esto es deliberado: un bloque singular no debería
  tumbar la simulación, aunque el resultado no sea una "inversa".
- `transpose() * M` en rotaciones da la identidad (invariante útil en tests).

### 2.4 Ejemplo compilable

```cpp
#include "phys2d/mat2.hpp"
#include <print>

int main() {
    using namespace phys2d;
    Mat2 r = Mat2::rotation(1.5707963f);      // 90°
    Vec2 v = r * Vec2{1.f, 0.f};             // (0, 1)
    std::println("r*v = ({:.3f},{:.3f})", v.x, v.y);
    std::println("det(r) = {:.4f}", r.determinant());   // 1

    Mat2 s = Mat2::scaling({2.f, 3.f});
    Vec2 w = s * Vec2{10.f, 10.f};            // (20, 30)
    std::println("w = ({:.1f},{:.1f})", w.x, w.y);
    std::println("s * s.inverse() == identity? {}",
                 (s * s.inverse()) == Mat2::identity());

    // Construcción: filas lógicas (1,2) / (3,4)
    Mat2 m(1.f, 2.f, 3.f, 4.f);
    std::println("row0=({:.1f},{:.1f}) row1=({:.1f},{:.1f})", m.row0().x, m.row0().y, m.row1().x, m.row1().y);
    return 0;
}
```

---

## 3. Formas — `include/phys2d/shapes.hpp` (787 líneas)

Todas las formas son **locales** al cuerpo. Constante `inline constexpr float kPi`.

```cpp
enum class BodyType : unsigned char { Static, Kinematic, Dynamic };
using Shape = std::variant<Circle, AABB, ConvexPolygon, Capsule, Segment>;
using Box = AABB; // alias histórico
```

- `BodyType` vive aquí (no en `body.hpp`) para evitar un include circular;
  `body.hpp` debe incluir `shapes.hpp` y **no** redefinirlo.
- `Shape` es un `std::variant` sin allocations (salvo `ConvexPolygon`, que
  contiene `std::vector<Vec2>`).
- `MassProps` (centro, masa, invMass, inercia, invInertia) se calcula por forma
  y densidad (`massProperties(shape, density, type)`, `shapeArea`, `boundingRadius`).

### 3.1 AABB / Box (caja axis-aligned en local)

```cpp
struct AABB {
    Vec2 lower{};
    Vec2 upper{};
};
```

Constructores estáticos:

```cpp
static constexpr AABB fromCenterHalf(Vec2 c, Vec2 h);       // centro ± half
static constexpr AABB fromCenterSize(Vec2 c, Vec2 s);       // centro ± size/2
static constexpr AABB fromLowerUpper(Vec2 lo, Vec2 hi);
```

Propiedades: `center()`, `half()`, `size()`, `width()`, `height()`, `area()`,
`perimeter()`, `isValid()` (`lower ≤ upper` por eje), `contains(point)`,
`contains(AABB)`, `intersects(AABB)`, `closestPoint(point)` (= `clamp`),
`expanded(margin)`, `combined(other)` (min–max de ambas), `testPoint(point)`.

> Una `Box` **no** tiene ángulo propio: la rotación es `Body::angle`. Si la
> rotas, su AABB en mundo es el de las 4 esquinas transformadas
> (`shapeAABB`, `transformShape`).

### 3.2 Circle

```cpp
struct Circle {
    float radius = 10.f;
};
```

- `area()` = `π·r²`, `perimeter()` = `2πr`, `testPoint(p)` = `|p| ≤ r`
  (usa `length2`), `closestPoint(p)` = `p` si está dentro, `p̂·r` si está
  fuera, `aabb()` = `[-r, r]` en cada eje.

### 3.3 ConvexPolygon

```cpp
struct ConvexPolygon {
    std::vector<Vec2> verts; // en espacio local
};
```

Requisitos: **convexo**, ≥ 3 vértices, área > `1e-8f`. La orientación
(antihoraria o horaria) **no** importa para validez — `signedArea()`, `area()`
y `edgeNormals()` la detectan por el signo — pero los tests del AV reportan
que la forma se valida con `isConvex()`/`isValid()`, y `World` lanza
`std::invalid_argument` si no lo es.

- `signedArea()` (shoelace, signado), `area()` (valor absoluto),
  `perimeter()`, `isConvex()` (todos los productos cruzados con el mismo
  signo salvo tolerancia `1e-8f`), `isValid()` (≥3 vértices, área > `1e-8f`
  y convexo).
- `centroid()`: centroide exacto si el área signada es suficiente, si no
  media aritmética de vértices.
- `massProps(density)`: inercia de polígono plano integrando por triángulos
  desde el origen y transportando al centroide (`I_centroide = I_origen − m·|c|²`).
- `transform(pos, angle)` ⇒ `std::vector<Vec2>` en mundo.
- `testPoint(p)`: mismo signo del producto cruzado en cada arista
  (tolerancia `−1e-6f`).
- `closestPoint(p)`: `p` si está dentro, si no la proyección sobre el
  segmento más cercano.
- `aabb(angle)` y `aabb(pos, angle)`: AABB mundo recorriendo vértices
  transformados.
- `edgeNormals()`: normales externas por arista
  (`(b−a).perpendicular().normalized() · (−signo)`, con `signo = ±1` según el
  área signada). El polígono **debe** ser convexo para que sean correctas.

### 3.4 Capsule

```cpp
struct Capsule {
    float radius = 10.f;
    float length = 20.f;      // altura total, incluyendo tapones
    Vec2 dir{0.f, 1.f};       // dirección; `configure` la normaliza (dir nulo/NaN ⇒ throw)
};
```

- `midLength()` = `length − 2·radius` si cabe, si no `0` (degenera a círculo).
- `capA()`/`capB()` = `dir·(±midLength/2)`; alias `a()`/`b()`.
- `area()` = `πr² + 2·r·midLength`.
- `closestAxisPoint(p)`: proyección sobre el segmento del eje.
- `closestPoint(p)` / `testPoint(p)`: distancia al eje ≤ `radius`.
- `aabb(angle)`, `aabb(pos, angle)`: envolvente de ambos tapones rotados.
- `Capsule::fromSegment(segA, segB, r)`: crea una cápsula cuyo eje va de
  `segA` a `segB`; si el segmento es nulo usa `dir = (0,1)`.

### 3.5 Segment

```cpp
struct Segment {
    Vec2 a{};
    Vec2 b{100.f, 0.f};
    float radius = 0.f;       // 0 = línea ideal, >0 = cápsula gruesa
};
```

- `delta()` = `b − a`, `length()` = `|delta|`, `direction()` = `delta/|delta|`
  (o `(1,0)` si es nulo), `normal()` = `direction().perpendicular()`.
- `closestPoint(p)` / `closestAxisPoint(p)`: proyección clampzada en `[0,1]`.
- `closestSurfacePoint(p)`: si `radius ≤ 0` es el eje; si no, el punto sobre la
  circunferencia de radio `radius` alrededor del eje, `p` si está dentro.
- `testPoint(p)`: con radio ≤ 0 usa `1e-4f` como tolerancia.
- `aabb(angle)`, `aabb(pos, angle)`: min/max de ambos extremos rotados
  expandidos por `radius`.

### 3.6 Params — inicializadores designados C++23

Cada forma tiene un struct `Params` plano (aggregate), pensado para
inicialización designada. **C++23 exige todo-designado o todo-posicional**:
no se puede mezclar `{{100.f, 50.f}, .radius = 20.f}`.

```cpp
// Todos estos son válidos:
phys2d::World w;
int id1 = w.addDynamicCircle(phys2d::CircleParams{{100.f, 50.f}, 20.f}); // posicional
int id2 = w.addDynamicCircle({.pos = {100.f, 50.f}, .radius = 20.f, .vel = {40.f, 0.f}});
//       ^^^^^^^^^^^^^^^^  (agregado, usa CircleParams como destino)
// Con variables intermedias se prefiere el tipo explícito:
phys2d::CircleParams cp{.pos = {100.f, 50.f}, .radius = 20.f};
int id3 = w.addDynamicCircle(cp);

int id4 = w.addDynamicBox({.pos = {400.f, 40.f}, .half = {40.f, 40.f}, .angle = 0.3f});
int id5 = w.addDynamicCapsule({.pos = {200.f, 60.f}, .radius = 8.f, .length = 24.f});
int id6 = w.addDynamicPolygon({.pos = {500.f, 80.f}, .verts = {{-20.f, -10.f}, {20.f, -10.f}, {0.f, 20.f}}});
int id7 = w.addStaticSegment({.pos = {0.f, 0.f}, .a = {0.f, 600.f}, .b = {1200.f, 600.f}});
```

Punto clave del diseño: `World` mapea

| Helper | Equivalente |
|---|---|
| `addDynamicCircle(CircleParams p)` | `p.type = Dynamic; addCircle(p)` |
| `addStaticBox(BoxParams p)` | `p.type = Static; addBox(p)` |
| `addKinematicPolygon(PolygonParams p)` | `p.type = Kinematic; addPolygon(p)` |

es decir, los helpers `addStatic*`/`addDynamic*`/`addKinematic*` **fuerzan**
`BodyType` sobre el campo `type` del `Params`, aunque se lo le pases distinto.
Para suelo, `BoxParams::fromLowerUpper({0, 590}, {1200, 610})` o
`fromSize(pos, sz)` (desde `BoxParams`).

Structs disponibles: `BodyParams`, `CircleParams`, `BoxParams`, `CapsuleParams`,
`PolygonParams`, `SegmentParams`, con los mismos campos comunes:
`pos`, `vel`, `angle`, `angVel`, `mass`, `friction`, `restitution`,
`linearDamping`, `angularDamping`, `type` (por defecto `BodyType::Dynamic`,
**salvo** `SegmentParams::type = Static` y los `addStatic*` que fuerzan Static).

### 3.7 Funciones libres sobre Shape

```cpp
[[nodiscard]] inline float shapeArea(Shape const& shape);
[[nodiscard]] inline float boundingRadius(Shape const& shape);
[[nodiscard]] inline MassProps massProperties(Shape const& shape, float density, BodyType type);
[[nodiscard]] inline Vec2 transformPoint(Vec2 local, Vec2 pos, float angle);   // localToWorld
[[nodiscard]] inline Vec2 localToWorld(Vec2 local, Vec2 pos, float angle);
[[nodiscard]] inline Vec2 worldToLocal(Vec2 world, Vec2 pos, float angle);
[[nodiscard]] inline Shape transformShape(Shape const& shape, Vec2 pos, float angle);
[[nodiscard]] inline AABB shapeAABB(Shape const& shape, Vec2 bodyPos, float bodyAngle);
[[nodiscard]] inline bool testPoint(Shape const& shape, Vec2 localPoint);
[[nodiscard]] inline bool pointInside(Shape const& shape, Vec2 worldPoint, Vec2 bodyPos, float bodyAngle);
[[nodiscard]] inline Vec2 closestPoint(Shape const& shape, Vec2 localPoint);
```

- `transformPoint(local) = local.rotated(angle) + pos`.
- `transformShape` para `AABB` envuelve las 4 esquinas en un AABB mundo nuevo;
  el resto conserva el tipo (cápsula rota `dir`, segmento mueve `a` y `b`).
- `boundingRadius`: radio de la esfera envolvente alrededor del origen local
  (la que usa broadphase y CCD adaptativo).

### 3.8 Ejemplos compilables

```cpp
#include "phys2d/phys2d.hpp"
#include <print>

int main() {
    using namespace phys2d;

    // Formas sueltas estilo lúdico
    Circle c{20.f};
    std::println("area circulo={:.1f}", c.area());
    std::println("testPoint(10,10)={}", c.testPoint({10.f, 10.f}));
    Vec2 cp = c.closestPoint({100.f, 0.f}); std::println("closestPoint(100,0)=({:.1f},{:.1f})", cp.x, cp.y);

    AABB box = AABB::fromCenterHalf({0.f, 0.f}, {30.f, 10.f});
    std::println("box center=({:.1f},{:.1f}) area={:.1f}", box.center().x, box.center().y, box.area());

    ConvexPolygon tri;
    tri.verts = {{-20.f, -10.f}, {20.f, -10.f}, {0.f, 20.f}};
    std::println("tri convex={} valid={} area={:.1f}", tri.isConvex(), tri.isValid(), tri.area());
    std::println("tri normals: {}", tri.edgeNormals().size());

    Capsule cap = Capsule::fromSegment({0.f, 0.f}, {0.f, 30.f}, 8.f);
    std::println("cap midLength={:.1f} area={:.1f}", cap.midLength(), cap.area());

    Segment seg{{0.f, 600.f}, {1200.f, 600.f}, 0.f};
    std::println("seg length={:.1f}", seg.length());

    // Masa por densidad
    MassProps props = massProperties(Shape{ConvexPolygon{tri.verts}}, 0.001f, BodyType::Dynamic);
    std::println("propiedades masa={:.3f} inertia={:.1f}", props.mass, props.inertia);
    return 0;
}
```

---

## 4. Body — `include/phys2d/body.hpp` (236 líneas)

Un cuerpo es un estado de movimiento más una `Shape`. Identificador:
`using BodyId = int;`.

```cpp
struct Body {
    Vec2 pos{};
    Vec2 vel{};
    Vec2 force{};
    float angle = 0.f;
    float angVel = 0.f;
    float torque = 0.f;
    float mass = 1.f;
    float invMass = 1.f;
    float inertia = 1.f;
    float invInertia = 1.f;
    float friction = 0.3f;
    float restitution = 0.2f;
    float linearDamping = 0.01f;
    float angularDamping = 0.01f;
    bool allowSleep = true;
    bool isSleeping = false;
    bool fixedRotation = false;
    bool bullet = false;
    float sleepTimer = 0.f;
    BodyType type = BodyType::Dynamic;
    Shape shape = Circle{};
};
```

Valores por defecto (círculo de radio 10): el `World` los sobrescribe al
configurar.

### 4.1 Tipo de cuerpo y dormido

```cpp
[[nodiscard]] constexpr bool isStatic() const noexcept;
[[nodiscard]] constexpr bool isKinematic() const noexcept;
[[nodiscard]] constexpr bool isDynamic() const noexcept;
[[nodiscard]] constexpr bool isAwake() const noexcept; // !isSleeping
```

Determinación de tipo (invariantes: `Dynamic` = tiene masa e inercia):

```cpp
void setStatic() noexcept;            // type + invMass = invInertia = 0, despierta
void setKinematic() noexcept;         // idem
void setDynamic(float m, float inertiaParam = 0.f) noexcept; // masa mínima 1e-8
void updateInertia() noexcept;        // recalcula inercia desde shapeArea + mass
void wake() noexcept;                 // restaura invMass/invInertia si Dynamic
void sleep() noexcept;                // vel = 0, invMass = invInertia = 0, sleepTimer = 0
```

- `setDynamic(m, inertia)`: si `inertia ≤ 1e-8f`, inercia = masa (esfera
  homogénea aproximada). Si `fixedRotation`, `invInertia = 0`.
- `updateInertia()`: densidad = `mass / shapeArea(shape)`, masa-inercia del
  `massProperties` transportada al centro, `inertia = props.inertia +
  mass·|center|²`. Si la forma tiene área nula o la masa es nula, inercia =
  masa (o 1). Solo afecta a `Dynamic`; en el resto `invInertia = 0`.
- `sleep()` pone `invMass = 0` y `invInertia = 0`: un cuerpo dormido es **no
  movible** en el solver hasta que se le despierte. `wake()` restaura los
  inversos desde `mass`/`inertia` si es Dynamic.
- `allowSleep` (por defecto `true`) es el opt-out individual: si es `false`,
  `World::updateSleep` lo salta aunque cumpla el criterio.

### 4.2 Fuerzas e impulsos

```cpp
void applyForce(Vec2 f) noexcept;                        // despierta si duerme
void applyForce(Vec2 f, Vec2 worldPoint) noexcept;       // + torque = r × f
void applyLinearImpulse(Vec2 imp) noexcept;              // y con punto (angVel)
void applyLinearImpulse(Vec2 imp, Vec2 worldPoint) noexcept;
void applyAngularImpulse(float imp) noexcept;
void clearForces() noexcept;                             // force = {}, torque = 0
```

- Todos los `apply*` despiertan si el cuerpo duerme (`isSleeping ⇒ wake()`).
- `applyForce` acumula `force`/`torque` (que `World::integrateForces` consume
  y **limpia al final de `step`**, ver `World::clearForces`). Não los
  re-apliques a mano en cada frame: un solo `applyForce` por paso de
  simulación o la fuerza se duplica acumulativamente.
- `applyLinearImpulse` es instantáneo sobre `vel` (+ `angVel` con punto);
  respeta `invMass`/`invInertia` — en `Static`/`Kinematic` o dormido es no-op
  (pero **sí despierta** el dormido aunque no tenga inversa al final del
  `wake()`; en estáticos no hay wake porque `setStatic` lo deja despierto).
- `force` y `torque` persisten hasta `clearForces()` o el final de `step`;
  `applyLinearImpulse` no pasa por `force`, modifica `vel` directamente.

### 4.3 Transformaciones local ↔ mundo

```cpp
[[nodiscard]] float boundingRadius() const noexcept; // bound de la forma
[[nodiscard]] Vec2 worldPoint(Vec2 local) const noexcept;  // local.rotated(angle)+pos
[[nodiscard]] Vec2 localPoint(Vec2 world) const noexcept;  // (world-pos).rotated(-angle)
```

Son los mismos cálculos que `transformPoint`/`worldToLocal`, pero con el
`pos`/`angle` del cuerpo.

### 4.4 `configure()` — configuración desde Params

Una overload por tipo de `Params` (`BodyParams`, `CircleParams`, `BoxParams`,
`CapsuleParams`, `PolygonParams`, `SegmentParams`):

- Aplica los campos comunes (`pos`, `vel`, `angle`, `angVel`, `friction`,
  `restitution`, `linearDamping`, `angularDamping`) y el estado por `type`
  (`setStatic`/`setKinematic`/`setDynamic`).
- Crea la `Shape` correspondiente y llama `updateInertia()`.
- `BodyParams` crea un `Circle{10.f}` por defecto.

`World` usa `configure()` en todos sus `add*`; no necesitas llamarla si sólo
usas `World`.

### 4.5 Ejemplo compilable

```cpp
#include "phys2d/phys2d.hpp"
#include <print>

int main() {
    using namespace phys2d;

    Body b;
    b.pos = {100.f, 50.f};
    b.shape = Circle{20.f};
    b.setDynamic(2.f);
    b.updateInertia();

    std::println("invMass={:.3f} inertia={:.1f}", b.invMass, b.inertia);
    Vec2 wp = b.worldPoint({20.f, 0.f}); std::println("worldPoint(20,0)=({:.1f},{:.1f})", wp.x, wp.y);
    Vec2 lp = b.localPoint({120.f, 50.f}); std::println("localPoint(120,50)=({:.1f},{:.1f})", lp.x, lp.y);

    b.applyForce({0.f, 100.f}, {120.f, 50.f}); // + torque
    std::println("force=({:.1f},{:.1f}) torque={:.1f}", b.force.x, b.force.y, b.torque);
    b.clearForces();

    b.applyLinearImpulse({30.f, 0.f});
    std::println("vel tras impulso=({:.1f},{:.1f})", b.vel.x, b.vel.y); // 30/2 = 15

    b.sleep();
    std::println("sleeping={} invMass={:.3f}", b.isSleeping, b.invMass);
    b.applyForce({0.f, 10.f}); // despierta solo
    std::println("tras applyForce: sleeping={}", b.isSleeping);
    return 0;
}
```

---

## 5. World — `include/phys2d/world.hpp`

`World` es el contenedor y el stepper. Copia y mueve: un `World` copiado
clona cuerpos **y** joints (`Joint::clone()`), un `World` movido transfiere
todo. **Doble ownership interno**: los cuerpos se almacenan como
`std::unique_ptr<Body>` en `bodies_`, y la API se mueve por punteros crudos
(`Body*`/`const Body*`) para iteradores y `bodiesSpan()`.

### 5.1 Configuración global

```cpp
Vec2 gravity{0.f, 900.f};        // px/s², Y hacia abajo
int velocityIterations = 8;      // solver secuencial post-integración
int positionIterations = 3;      // corrección posicional lineal + angular
int substeps = 1;                // mínimo de substeps por step (ver §5.4)
float cellSize = 64.f;           // celda SpatialHash (px)
BroadPhaseType broadPhaseType = BroadPhaseType::SpatialHash; // o SAP
float sleepVelocity = 1.0f;      // px/s y rad/s que habilitan dormir
float sleepTime = 0.5f;          // s bajo el umbral antes de dormir
bool bullet = false;             // global (cada Body también tiene su flag)
```

Alias: `World::BroadPhaseType = phys2d::BroadPhaseType`.
`enum class BroadPhaseType : unsigned char { SpatialHash, SAP };`.

Métodos de ajuste: `setBroadPhase(bp)`, `setBroadPhaseType(bp)`, `clear()`
(vacía todo, incluidos contactos y joints).

> `gravity` es un **campo público**: se asigna entre steps (`w.gravity = {0,900}`).
> No hay setter; el valor se lee en cada `stepOnce`.

### 5.2 Creación de cuerpos — `add*`

Todas devuelven **`int` = `BodyId`** (índice de `bodies_`, estable mientras no
borras/reinsertas en mitad de una secuencia). Se consumen vía
`World::body(id)`.

Dos familias por forma: overload posicional (`addCircle(pos, radius, vel, mass,
type)`, `addBox(pos, half, vel, mass, angle, type)`, `addPolygon(pos, verts,
vel, mass, angle, type)`, `addCapsule(pos, radius, length, vel, mass, angle,
type)`, `addSegment(pos, a, b, radius, vel, mass, angle, type)`) y overload con
`Params` (`addCircle(CircleParams p)`, etc.). Las dos familias comparten
defecto: `friction = 0.3f`, `restitution = 0.2f`, `linearDamping = 0.01f`,
`angularDamping = 0.01f`, `type = Dynamic` (Segment por defecto Static).

Y las "atalhos" por tipo:

```cpp
int addStaticCircle(Vec2 pos, float radius);   int addStaticCircle(CircleParams p);
int addStaticBox(Vec2 pos, Vec2 half, float angle = 0.f); int addStaticBox(BoxParams p);
int addStaticPolygon(PolygonParams p);         int addStaticCapsule(CapsuleParams p);
int addStaticSegment(SegmentParams p);

int addKinematicCircle(Vec2 pos, float radius, Vec2 vel = {}); int addKinematicCircle(CircleParams p);
int addKinematicBox(Vec2 pos, Vec2 half, Vec2 vel = {}, float angle = 0.f); int addKinematicBox(BoxParams p);
int addKinematicPolygon(PolygonParams p);     int addKinematicCapsule(CapsuleParams p);
int addKinematicSegment(SegmentParams p);

int addDynamicCircle(Vec2 pos, float radius, Vec2 vel = {}, float mass = 1.f); int addDynamicCircle(CircleParams p);
int addDynamicBox(Vec2 pos, Vec2 half, Vec2 vel = {}, float mass = 1.f, float angle = 0.f); int addDynamicBox(BoxParams p);
int addDynamicPolygon(Vec2 pos, std::vector<Vec2> verts, Vec2 vel = {}, float mass = 1.f, float angle = 0.f); int addDynamicPolygon(PolygonParams p);
int addDynamicCapsule(Vec2 pos, float radius, float length, Vec2 vel = {}, float mass = 1.f, float angle = 0.f); int addDynamicCapsule(CapsuleParams p);
int addDynamicSegment(SegmentParams p);
```

Los `addStatic*`/`addKinematic*`/`addDynamic*` **fuerzan** `BodyType`
respectivamente aunque el `Params` traiga otro valor de `type`. Idéntico con
`addSegment`, cuyo `type` por defecto es `Static` (suelo).

Genérico:

```cpp
int addBody(BodyParams p);                  // círculo r=10 por defecto
int addBody(BodyParams p, Shape shapeValue); // BodyParams + Shape explícita
```

`validateBody` lanza `std::invalid_argument` ante forma degenerada
(radio ≤ 0, `length ≤ 0` en Segment/Capsule, polígono < 3 vértices, no convexo,
área ≤ 1e-8f, Segments de longitud cero, AABB inválido, vértices de polígono
no finitos, `dir` de cápsula nulo/NaN, `radius < 0` en Segment), valores no
finitos (NaN/inf en `pos`, `vel`, `angle`, `angVel`, masa, amortiguamientos...)
o rangos físicos inválidos (fricción < 0, restitución fuera de [0, 1]).
Una `add*` que tira no registra el cuerpo.

#### Joints (en la API de World, porque viven en World)

```cpp
int addJoint(std::unique_ptr<Joint> joint);
template <typename JointT, typename... Args> int addJoint(Args&&... args);
int addDistanceJoint(int a, int b, float rest, Vec2 anchorA = {}, Vec2 anchorB = {});
int addRevoluteJoint(int a, int b, Vec2 anchorA, Vec2 anchorB);
int addWeldJoint(int a, int b, Vec2 anchorA, Vec2 anchorB);
int addPrismaticJoint(int a, int b, Vec2 axis, float lower, float upper, Vec2 anchorA = {}, Vec2 anchorB = {});
int addSliderJoint(int a, int b, Vec2 axis, float lower, float upper, Vec2 anchorA = {}, Vec2 anchorB = {});
int addPulleyJoint(int a, int b, Vec2 anchorA, Vec2 anchorB, Vec2 groundA, Vec2 groundB, float ratio);
int addSpringJoint(int a, int b, float rest, float stiffness, float damping, Vec2 anchorA = {}, Vec2 anchorB = {});
int addRopeJoint(int a, int b, float maxLength);
int addGearJoint(int a, int b, float ratio);
```

`addJoint` devuelve el `int` del joint; lanza `std::invalid_argument` si el
`unique_ptr` es nulo o `a == b`, y `std::out_of_range` si `a`/`b` no son
índices válidos de cuerpos.

### 5.3 Acceso a cuerpos y contactos

```cpp
[[nodiscard]] std::span<const Body*> bodiesSpan() const noexcept;  // raw ptrs, solo lectura
[[nodiscard]] Body& body(int id);                            // lanza std::out_of_range si id inválido
[[nodiscard]] Body const& body(int id) const;
[[nodiscard]] std::size_t jointCount() const noexcept;       // nº de joints vivos
[[nodiscard]] Joint& joint(int id);                          // lanza std::out_of_range si índice inválido
[[nodiscard]] Joint const& joint(int id) const;              // idem const
[[nodiscard]] std::span<const Manifold> contacts() const noexcept;  // contactos del último step
[[nodiscard]] std::span<const Manifold> contacts() noexcept;
[[nodiscard]] std::size_t size() const noexcept;
[[nodiscard]] bool empty() const noexcept;
void clear() noexcept;                                       // cuerpos + joints + contactos + buffers

// Iteración rango `for (auto& b : world)`:
BodyIterator begin(); BodyIterator end();
BodyConstIterator begin() const; BodyConstIterator end() const;
BodyConstIterator cbegin() const; BodyConstIterator cend() const;
```

- `body(id)` **lanza `std::out_of_range`** si `id < 0` o `≥ size()`. Nunca
  silencia: o id correcto o excepción.
- `bodiesSpan()` devuelve `std::span<const Body*>` sobre punteros crudos ya
  cacheados: es lo que usan `raycast`/`pointQuery` sin construir vector
  intermedio, y es de **solo lectura**. Para mutar, `body(id)` o la iteración
  del propio `World`.
- `joint(id)` devuelve la referencia al `Joint` (típicamente para ajustar
  parámetros: `static_cast<Revolute&>(world.joint(j)).motorSpeed = 2.f`) y
  lanza `std::out_of_range` si el índice no existe. `jointCount()` da el
  número de joints vivos.
- Iteración por rango sobre el propio `World` (`for (auto& b : world)`) recorre
  `Body&` y `const Body&` según el `World` sea mutable o no.

### 5.4 `step` — integración, substeps adaptativos, solver

```cpp
void step(float dt);
void step(float dt, int substepsArg);
```

- `step(dt)`: usa `World::substeps` como base.
- `step(dt, substepsArg)`: usa `substepsArg` si `> 0`, si no `World::substeps`.
- **`dt` no finito o `≤ 0` ⇒ `std::invalid_argument`** (antes silencioso).
  `dt` grande **no** se trunca más: se subdivide por substeps adaptativos
  (tope 128), lo que reparte el coste sin fingir que el tiempo no pasó.
- **Substeps adaptativos**: calcula por cuerpo no estático/dormido su
  desplazamiento por paso — `disp = max(|vel|·dt, |angVel|·r·dt)` con
  `r = boundingRadius` — y el `maxRatio = max(disp / r)` sobre todos. Si
  `maxRatio > 0.5`, subdivide en `ceil(maxRatio / 0.5)` substeps (tope 128).
  El número final es `max(base, adaptive)`, capado `[1, 128]`, y `h = dt / n`.
  Un cuerpo nunca avanza más de medio radio por substep: es la red de
  seguridad anti-túnel junto al CCD.

Cada substep (`stepOnce(h)`) hace, en orden:

1. `radii_[i] = bodies[i]->boundingRadius()` (cachea; se usa en broadphase,
   contactos y CCD).
2. `integrateForces(h)`: `vel += (gravity + force·invMass)·h`,
   `angVel += torque·invInertia·h`, damping `vel /= (1 + h·linearDamping)`,
   `angVel /= (1 + h·angularDamping)`, y `fixedRotation ⇒ angVel = 0`.
   **Solo `Dynamic` y no dormidos.**
3. **Broadphase**: `pairsSAP()` o `pairsSpatialHash()` (ver §5.5).
4. `findContacts()`: narrowphase sobre los pares, con persistencia de
   manifolds (si dos cuerpos no se movieron desde el step anterior — epsilon
   sub-píxel — reutiliza el manifold sin re-colisionar). Despierta a un
   dormido si su pareja está despierta **y moviéndose** (misma regla que los
   joints).
5. Los callbacks `onBeginContact`/`onEndContact` se disparan **una vez por
   step completo** (no por substep), solo para pares con al menos un cuerpo
   dinámico; los estático-estático no generan eventos.
5. **Solver secuencial de impulsos con torque** (ver §5.6):
   preparación de joints (despierta dormidos si su pareja se mueve),
   `velocityIterations` iteraciones de velocidad, corrección posicional con
   `positionIterations`, **CCD/swept** y `updateSleep`.
6. Persistencia de contactos para el siguiente paso (`prevContacts_`).

Al volver de `step`, **`clearForces()`** pone `force = {}` y `torque = 0` en
todos los cuerpos. Por eso `applyForce` debe re-aplicarse por step si quieres
fuerza sostenida (empujones puntuales mejor con impulso).

Valores típicos: `dt = 1/60f`, `velocityIterations = 8`, `positionIterations = 3`.
Para pilas estables, sube `substeps` o `velocityIterations`; para partículas
rápidas, `bullet` y/o `substeps` grandes.

#### Parada anti-túnel y CCD (dos capas)

1. **Swept con time-of-impact**: `sweepFastBodies(h)` detecta cuerpos
   dinámicos despiertos cuya `vel·h` supera su `boundingRadius` **o** con
   `bullet`, calcula el TOI y los coloca en la superficie de impacto con un
   impulso de restitución. Exacto en círculo–círculo (cuadrática),
   círculo sobre AABB sin rotación (slab) y convexo–convexo (AABB/polígono,
   diferencia de Minkowski); círculo contra cualquier otra forma usa
   `shapeCastBody` engordando el objetivo con el radio del círculo (círculo
   objetivo: `raycastCircle`; cápsula/segmento: `raycastCapsule`). Tras el
   impacto el cuerpo conserva el movimiento restante del substep
   `(1 - t)·h` con la velocidad post-impulso. El punto de impacto para el
   impulso se calcula sobre la superficie del movible con
   `closestPointOnShape` (no sobre el casco de Minkowski).
2. **Substeps adaptativos** como red (§5.4): el paso se subdivide para que
   ningún cuerpo se desplace más de medio radio.

Si una bala a `500 000 px/s` impacta contra una pared, se detiene en la
pared. Si no usas `bullet` y el cuerpo se mueve más que su radio, el CCD
automático lo captura en la mayoría de configuraciones.

### 5.5 Broadphase — SpatialHash o SAP

```cpp
World::setBroadPhase(BroadPhaseType::SpatialHash); // por defecto
World::setBroadPhase(BroadPhaseType::SAP);
World::cellSize = 64.f; // solo afecta a SpatialHash
```

Ambos retornan **parejas candidatas ordenadas sin duplicados** (`pairs_`),
y `findContacts()` aplica el filtro de círculo envolvente
(`|posA−posB| > sumR ⇒ sigue`) antes de la narrowphase.

| Aspecto | SpatialHash | SAP |
|---|---|---|
| Estructura | rejilla de celdas (`cellSize`) hasheadas por `(cx, cy)` | endpoints `x − r` / `x + r` ordenados por X |
| Construcción | `buildGrid()`: una entrada por celda que toca el círculo `pos ± r`; ordenación radix LSD estable por clave | `eps_` ordenado con `std::sort`; escaneo con `active_` |
| Pareo | dentro de cada celda, `O(k²)`; luego radix + `unique` | al abrir un endpoint, pareo contra `active_` con filtro `\|dy\| ≤ r1+r2` |
| Coste | con muchos objetos grandes, muchas entradas por cuerpo | con objetos casi en la misma X, `active_` crece (peor caso O(n²)) |
| Ajuste | `cellSize`: celda demasiado grande ⇒ muchos candidatos; demasiado pequeña ⇒ cuerpos grandes ⇒ muchas entradas | no hay parámetro; es eficiente si la escena se extiende uniformemente en X y hay movimiento horizontal |

Notas de implementación:

- La **retención de contactos** no afecta al broadphase: `SpatialHash`
  no tiene warm starting de grid, cada step reconstruye `gridEntries_`.
- SpatialHash usa **radix sort LSD** (8 pasadas, 8 bits por pasada) tanto
  para celdas como para parejas, evitando un `unordered_map`: las entradas de
  una celda quedan contiguas, y la estabilidad preserva `a < b`. El
  rendimiento medido es ~1.2 ms/step con 500 cuerpos y ~6.5 ms/step con 2000
  en pilas de círculos (coste crece con nº de contactos, no solo de cuerpos).
- SAP es `O(n log n)` en ordenamiento con memoria `O(n)` y coste de escaneo
  proporcional a solapes en X. Para escenas con X dispersa suele ganar; con el
  eje más disperso en Y, rota ejes visualmente — como no es configurable en
  esta versión, se usa X siempre.

### 5.6 Solver secuencial con torque (hasta 2 puntos por manifold)

Cada `Manifold` se expande a uno o dos `SolverContact` (**un por punto**):
así el impulso en cada punto genera el torque correcto sobre ambos cuerpos.
`effectiveMass(armA, armB, dir)` usa `k = invMassA + invMassB +
invInertiaA·(armA×dir)² + invInertiaB·(armB×dir)²`.

- `integrateForces` (integración semi-implícita de Euler con damping lineal
  implícito), `velocityIterations` pases de velocidad (fricción `conjunta =
  sqrt(fricA·fricB)`, restitución `restA·restB`), `positionIterations` de
  corrección posicional lineal + angular.
- Warm starting: impulsos del step anterior se reutilizan como semilla.
- Persistencia exacta de manifolds: pares cuyos cuerpos no se movieron
  reutilizan el manifold sin re-colisionar.
- Corrección posicional separa el **contacto** y la **penetracción**: si
  hay contacto sostenido (contacto +|penetra|), la corrección es más suave.

Esto no es API pública salvo por `contacts()`: los manifolds devueltos
contienen `normal` (Vec2, de A a B), `point`/`point2` (Vec2), `count` (int, 1 o 2), `depth` (float), `a`, `b`,
`restitution`, `friction`, `relVel`.

### 5.7 Sleep global

`World::updateSleep(h)` por cuerpo dinámico **con `allowSleep`** y no dormido:

```cpp
bool slow = |vel| < sleepVelocity && |angVel| < sleepVelocity;
if (slow) sleepTimer += h; else sleepTimer = 0;
if (sleepTimer >= sleepTime) sleep();   // vel = 0, angVel = 0, invMass = invInertia = 0
```

Parámetros por defecto: `sleepVelocity = 1.0f` (el ruido residual de un
cuerpo en reposo en este solver ronda 0.5 px/s; 0.05 nunca llegaría a dormir),
`sleepTime = 0.5f`.
Comportamiento:

- Un estático/kineático nunca duerme aunque sea lento.
- Un `allowSleep = false` nunca duerme.
- Despertares: `applyForce`/`apply*Impulse` (llaman `wake()`),
  contacto con un cuerpo despierto **que se mueve** — donde movimiento
  significa `preVelocidad > sleepVelocity` medida con la velocidad del
  **substep anterior** (o `|angVel| > sleepVelocity`); la gravedad
  integrada en el propio substep no cuenta como movimiento —, joints cuyo
  compañero se mueve (misma regla), CCD impactando contra un dormido.
- Los dormidos no integran fuerzas ni se mueven; `isSleeping = true` y
  `invMass = 0`. Es como un estático hasta que algo lo despierte.

> Para objetos que nunca deben dormir (p. ej. plataformas cinemáticas
> lentas), pon `allowSleep = false` o no pongas `Dynamic` sin masa.

### 5.8 Callbacks — `onBeginContact` / `onEndContact`

```cpp
using Callback = std::function<void(const Manifold&)>;
void onBeginContact(Callback cb); // no-op si cb es nula
void onEndContact(Callback cb);
```

- Se registran en `beginCallbacks_` / `endCallbacks_` y se invocan
  **dentro del step** (tras la actualización de contactos, comparando
  `curGeom_` contra `prevGeom_`). Un callback nulo no se registra.
- `Manifold` recibido en `onBeginContact` es un **contacto nuevo en este
  step**; en `onEndContact` un contacto que existía y ya no.
- La firma es `const Manifold&`: no guardes la referencia, cópiala si la
  necesitas fuera del callback.
- Determinación por clave `(a, b)` con `a < b`: dentro de un manifold
  `a` y `b` son ids de cuerpos (`BodyId`), no índices en `contacts()`.

```cpp
phys2d::World world;
world.onBeginContact([](const phys2d::Manifold& m) {
    if (m.a == world.body(hitGroundId).pos.x > 0 && /* ... */) {}
});
```

Ejemplo completo y seguro:

```cpp
#include "phys2d/phys2d.hpp"
#include <print>
#include <set>

int main() {
    phys2d::World world;
    int floor = world.addStaticBox({.pos = {400.f, 590.f}, .half = {500.f, 10.f}});
    int ball = world.addDynamicCircle({.pos = {100.f, 50.f}, .radius = 20.f, .vel = {40.f, 0.f}});

    std::set<std::pair<int, int>> active;
    world.onBeginContact([&](const phys2d::Manifold& m) {
        active.insert({m.a, m.b});
        std::println("begin: {} - {} (n=({:.1f},{:.1f}), depth={:.1f})", m.a, m.b, m.normal.x, m.normal.y, m.depth);
    });
    world.onEndContact([&](const phys2d::Manifold& m) {
        active.erase({m.a, m.b});
        std::println("end: {} - {}", m.a, m.b);
    });

    for (int i = 0; i < 180; ++i) {
        world.step(1.f / 60.f);
    }
    std::println("contactos activos al final: {}", active.size());
    (void)floor; (void)ball;
    return 0;
}
```

### 5.9 Fuerzas e impulsos a nivel World

```cpp
void applyForce(int id, Vec2 force);
void applyForce(int id, Vec2 force, Vec2 worldPoint);   // produce torque
void applyLinearImpulse(int id, Vec2 imp);
void applyAngularImpulse(int id, float imp);
void clearForces(); // limpia todos los cuerpos
```

Delegan en `body(id).apply*(…)`, luego son sujetos a `std::out_of_range` si
el id no existe. La `worldPoint` es en **espacio mundo**
(`torque += (worldPoint − pos).cross(force)`).

> `World` no tiene `applyForce(int, Vec2, Vec2)` para el punto **local**: no
> existe en la API. Convierte antes con `body(id).worldPoint(local)`.

### 5.10 Queries — raycast

```cpp
[[nodiscard]] RayHit raycast(Vec2 origin, Vec2 dir, float maxDist) const;
[[nodiscard]] bool raycast(Vec2 origin, Vec2 dir, float maxDist, Vec2& hitPoint, int& hitBody) const;
```

- `RayHit` tiene `hit` (`bool`), `point` (`Vec2`), `bodyId` (`int`), `normal`
  y `fraction` (`t / maxDist`, 0 si no hay hit, típicamente `1.f`).
- `dir` **no** tiene que estar normalizado: el test usa proyección y
  `fraction` normalizada por `maxDist`.
- `raycast(origin, dir, maxDist, hitPoint, hitBody)` devuelve `bool` y
  rellena los out-params.
- Existe también `raycastPoint(origin, dir, maxDist, w.bodiesSpan())`
  (versión libre en `query.hpp` sobre `std::span<const Body*>`) y
  `pointQuery(point, span)` — ver `query.hpp` para la forma libre. El
  alcance del rayo incluye el punto a `maxDist` exacto (hit con
  fracción 1).

```cpp
phys2d::World world;
auto hit = world.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f);
if (hit.hit) {
    // hit.point, hit.normal, hit.bodyId, hit.fraction
    std::println("hit {} en {}", hit.bodyId, hit.point);
}
```

### 5.11 Ejemplo compilable — el "ultra fácil"

```cpp
#include "phys2d/phys2d.hpp"
#include <print>

int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    int floor = world.addStaticBox(
        phys2d::BoxParams{.pos = {400.f, 590.f}, .half = {500.f, 10.f}});
    int ball = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {100.f, 50.f}, .radius = 20.f, .vel = {40.f, 0.f}});

    world.addJoint<phys2d::Distance>(floor, ball, 150.f); // opcional

    for (int i = 0; i < 120; ++i) {
        world.step(1.f / 60.f);
    }

    auto hit = world.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f);
    if (hit.hit) {
        std::println("hit cuerpo {} en ({:.1f},{:.1f})", hit.bodyId, hit.point.x, hit.point.y);
    }
    for (auto& b : world) {
        std::println("pos=({:.1f},{:.1f}) vel=({:.1f},{:.1f})", b.pos.x, b.pos.y, b.vel.x, b.vel.y);
    }
    (void)floor; (void)ball;
    return 0;
}
```

Compilación:

```bash
g++ -std=c++23 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror \
  -I include cpp/main.cpp -o main
# header-only: no hay que enlazar ninguna librería, solo los flags
```

---

## 6. Errores frecuentes (y qué hace phys2d en cada caso)

### 6.1 Inicialización con `Params` mal formada

| Error | Efecto |
|---|---|
| Mezcla `{{100,50}, .radius=20}` | **No compila** (C++23: todo-posicional o todo-designado). Usa `{{100.f,50.f}, 20.f, {40.f,0.f}}` o `{.pos={100.f,50.f}, .radius=20.f, .vel={40.f,0.f}}`. |
| `Vec2{100, 50}` sin `.f` | Compila con conversión `double→float` y warning estrictos (`-Wconversion` lo señala). Usa siempre `100.f`. |
| `w.addDynamicCircle({.pos={100.f,50.f}, .radius=20.f})` sin tipo previo | Se deduce `CircleParams` por el parámetro. Funciona si el destino es `CircleParams`. Con variables intermedias escribe el tipo: `CircleParams cp{...}; w.addDynamicCircle(cp);` |

### 6.2 Cuerpos

| Error | Efecto |
|---|---|
| `world.body(999)` no existe | Lanza `std::out_of_range("phys2d::World::body: id fuera de rango")`. |
| Radio ≤ 0, polígono no convexo, área nula, masa ≤ 0, NaN/inf en campos | `registerBody`/`validateBody` lanza `std::invalid_argument` antes de registrar el cuerpo. |
| Hacer `push_back` directo sobre el almacenamiento interno de cuerpos | No es posible por diseño: no hay acceso público a `bodies_`. Pasa siempre por `add*` para que `validateBody` compruebe y se cacheen punteros y radios. |
| Cambiar `body.shape` a mano sin `updateInertia()` | La inercia queda obsoleta. Llama `b.updateInertia()` tras tocar la forma, o usa `World::addBox(p, shape)`. |
| `Shape` con `Box` pensando que tiene ángulo | No lo tiene. Pon el ángulo en `Body::angle` (o en `BoxParams::angle`). |
| Polígono con vértices en orden incorrecto o auto-intersecado | `isConvex()` devuelve falso ⇒ `isValid()` falso ⇒ excepción. Revierte el orden o triangulariza. |

### 6.3 Fuerzas / step

| Error | Cómo se nota |
|---|---|
| `step(0.f)` o `step(-0.1f)` | Lanza `std::invalid_argument` (nada se simula ni se toca). |
| `step(0.1f)` | No hay clamp: se subdivide por substeps adaptativos (tope 128). Sin sorpresas, pero 0.1s a 60 Hz sigue siendo un frame enorme — usa `1/60` o substeps. |
| Aplicar `applyForce` una vez por frame mientras tu loop tiene varios `step` | Cada `step` consume y limpia las fuerzas. Si el loop hace 2 `step` por frame, sobran las fuerzas en el segundo. |
| Asumir que `applyForce` se acumula entre `step` | No. `step` limpia `force`/`torque` al final. Lo que cambia persistentemente es `vel` (impulsos) o `pos` (mover directamente). |
| `applyLinearImpulse` sobre un estático | No-op (invMass 0). |
| Operar con id de un cuerpo eliminado por `clear()` | `body(id)` lanza `std::out_of_range` (el vector está vacío). |

### 6.4 Sleep / broadphase / bullets

| Error | Efecto |
|---|---|
| Querer pilares estáticos que no duerman | `allowSleep = false` en el cuerpo (cuerpo por cuerpo) o sube `sleepTime` a un valor alto. `sleepTime` es global, `allowSleep` individual. |
| Un dinamico duerme en mitad de vuelo | Su velocidad está por debajo de `sleepVelocity` antes de caer — si estás en caída libre te afectó la gravedad la suficiente. (Quizás estás en fricción con suelo.) |
| CCD aproximado para cápsulas/segmentos | Barrido del centro con `shapeCastBody` (círculo/cápsula/segmento engordan el objetivo; polígono y AABB rotado son aproximación por el centro). Pon `bullet = true` para forzar el barrido. |
| `world.broadPhaseType = SAP` antes de añadir cuerpos | Estable, no reordena nada. Redimensiona al registrar `radii_` por step. |

### 6.5 Callbacks y consultas

| Error | Efecto |
|---|---|
| Guardar `const Manifold&` del callback para luego | La referencia es válida solo durante la llamada (una pila local por step). Copia el `Manifold` entero o los campos que necesites. |
| Registrar un callback con `std::function{nullptr}` | No-op (explicitamente no se añade). |
| Asumir `(a, b)` del Manifold en el orden de inserción | Siempre `a < b` (clave ordenada). En efectos, distingue las partes por `body(a).pos` contra `body(b).pos`. |
| Contar subpasos/eventos de un contacto en un step | Los callbacks disparan **una vez por step** (no por substep) y los pares estático-estático no los disparan. |
| `raycast` con `dir = Vec2{0,0}` | Retorna `RayHit{hit=false}`. No lances excepción, pero no "detected". |
| Añadir/quitar callbacks desde dentro de un callback | Se acepta: los callbacks se copian una vez por step; los añadidos se usan en el siguiente step. |

### 6.6 Copias / movimientos

```cpp
phys2d::World a;
a.addDynamicCircle({.pos = {100.f, 50.f}, .radius = 20.f});
phys2d::World b = a;   // clona cuerpos y joints (Joint::clone())
phys2d::World c = std::move(a); // a queda vacío
```

Un `World` copiado **no** comparte memoria de cuerpos (`unique_ptr`): no
hay aliasing. Los ids se conservan (mismo orden, mismo `BodyId`).

### 6.7 Directrices mínimas de estilo para no romper la API

- `Vec2`/`Mat2` siempre con `.f` en literales. El project compila con
  `-Wconversion -Wshadow -Werror`.
- `Box = AABB` alias histórico: `AABB` es el nombre definitivo en el código.
- Coordenadas en píxeles, `+Y` hacia abajo. Ángulos en radianes.
- Tareas químicas (ensamblar un nivel, debug draw) no pertenecen al `World`:
  son código de tu "juego". `World` sólo guarda y simula cuerpos/joints.
- `Body::pos` es el estado publico: mutarlo directamente es válido y lo hace
  el stepper. Si lo haces entre frames, el siguiente `step` usará la nueva
  posición (y `findContacts` recalculará parejas con el broadphase).

---

## 7. Referencia rápida de símbolos

| Símbolo | Fichero | Uso |
|---|---|---|
| `phys2d::Vec2` | `vec2.hpp` | vectores 2D en píxeles |
| `phys2d::Mat2` | `mat2.hpp` | matrices 2×2 columna-major |
| `phys2d::AABB` / `Box` | `shapes.hpp` | caja axis-aligned / alias |
| `phys2d::Circle` | `shapes.hpp` | círculo |
| `phys2d::ConvexPolygon` | `shapes.hpp` | polígono convexo |
| `phys2d::Capsule` | `shapes.hpp` | cápsula («pile caps») |
| `phys2d::Segment` | `shapes.hpp` | segmento (línea o cordel grueso) |
| `phys2d::Shape` | `shapes.hpp` | `variant<Circle, AABB, ConvexPolygon, Capsule, Segment>` |
| `phys2d::BodyType` | `shapes.hpp` | `Static` / `Kinematic` / `Dynamic` |
| `phys2d::MassProps` | `shapes.hpp` | centro, masa, invMass, inercia, invInertia |
| `phys2d::BodyParams` y demás `*Params` | `shapes.hpp` | inicializadores designados C++23 |
| `shapeArea`, `boundingRadius`, `massProperties` | `shapes.hpp` | libres sobre `Shape` |
| `transformPoint`, `localToWorld`, `worldToLocal` | `shapes.hpp` | conversión local↔mundo |
| `phys2d::Body`, `BodyId` | `body.hpp` | estado de movimiento |
| `phys2d::World` | `world.hpp` | contenedor + stepper |
| `phys2d::BroadPhaseType` | `world.hpp` | `SpatialHash` / `SAP` |
| `phys2d::World::add*` | `world.hpp` | alto-level factory por tipo |
| `World::step(dt)` / `step(dt, substeps)` | `world.hpp` | integra, broadphase, narrowphase, solver, sleep, CCD |
| `World::onBeginContact` / `onEndContact` | `world.hpp` | callbacks de contactos |
| `World::raycast` | `world.hpp` | rayo 2D contra cuerpos |
| `World::bodiesSpan()`, `size()`, `clear()` | `world.hpp` | acceso e iteración |
| `World::jointCount()`, `World::joint(id)` | `world.hpp` | joints por índice (`std::out_of_range`) |
| `World::applyForce` / `applyLinearImpulse` / `applyAngularImpulse` | `world.hpp` | helpers por id |

Mismo espacio de nombres: `phys2d`. Todos los header-only sin macros ni
allocs en el hot path (salvo `ConvexPolygon::verts` y `World` internamente,
que re-utilizan buffers y no re-aloyen en absoluto tras `step` calientes).
