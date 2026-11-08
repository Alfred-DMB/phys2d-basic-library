# Queries y DebugDraw — phys2d

Documentación extensa de `include/phys2d/query.hpp` y `include/phys2d/debug.hpp`,
más los ejemplos y el benchmark que las ejercitan. Todas las consultas viven en
el namespace `phys2d`, son **header-only** (solo `inline`), C++23, compilables con
`-Werror` y **no dependen de `World`**: operan sobre primitivas geométricas o sobre
conjuntos de `Body` pasados como `span`. Eso las hace reutilizables para picking,
IA, depuración, disparo, barridos anti-tunneling y tests.

> Convención general de `query.hpp`: el vector `dir` de todo rayo/barrido no
> necesita venir normalizado; la implementación lo normaliza internamente.
> `maxDist` es la longitud máxima (en unidades del mundo) que se inspecciona.
> El resultado usa `RayHit::fraction = distancia / maxDist` (en `[0,1]` cuando
> hay impacto), de modo que el punto es `origin + dirNormalizado * (fraction * maxDist)`.

---

## 1. Tipos base

### 1.1 `RayHit` / `RayCastResult`

```cpp
struct RayHit {
    bool hit = false;
    Vec2 point{};
    Vec2 normal{};
    int bodyId = -1;
    float fraction = 1.f;
};
using RayCastResult = RayHit;
```

| Campo | Significado |
|---|---|
| `hit` | `true` si el rayo/barrido impacta dentro de `maxDist`. En `false`, el resto de campos no tienen significado (valores por defecto). |
| `point` | Punto de impacto en coordenadas de mundo. Si el rayo nace **dentro** de la forma, es el `origin` (ver §3). |
| `normal` | Normal en el punto, apuntando **hacia fuera** y contra la dirección del rayo (`dot(normal, dir) <= 0` en impacto frontal). Si nace dentro, es `-dir` normalizado. |
| `bodyId` | Índice del cuerpo impactado dentro del `span` recorrido (0-based). Solo lo rellenan las consultas sobre conjuntos (`raycastPoint`, `raycastWorld`, `shapeCastPoint`); en consultas primitivas o sobre un único `Body` queda en `-1`. |
| `fraction` | `t / maxDist`, donde `t` es la distancia desde `origin` al impacto a lo largo de la dirección normalizada. `0` = impacto en el origen; `1` = impacto justo en el límite. |

Valores de `hit=false` se obtienen con `RayHit{}` (defecto).

### 1.2 Spans de cuerpos (compatibilidad con `World`)

```cpp
using BodySpan    = std::span<std::unique_ptr<Body> const>;
using BodyPtrSpan = std::span<Body const*>;
```

`query.hpp` y `debug.hpp` aceptan ambos sin adaptadores:

- `World::bodiesSpan()` devuelve `std::span<const Body*>` (punteros crudos,
  de solo lectura). Se pasa a los overloads que piden `BodyPtrSpan`:
  ```cpp
  phys2d::RayHit h = phys2d::raycastPoint(o, d, maxD, w.bodiesSpan());
  ```
- `std::span<const std::unique_ptr<Body>>` (vectores propios del llamante)
  se acepta vía los overloads `BodySpan`.
- Los `nullptr` se saltan defensivamente.

`bodyId` en las consultas sobre conjunto es el índice en el span, **no**
necesariamente el id de `World` (aunque en la práctica coinciden porque `World`
almacena los cuerpos en orden de creación y nunca reordena su vector).

---

## 2. AABB y solapamiento — `overlapAABB`, `bodyAABB`

### 2.1 `overlapAABB` (tres overloads)

```cpp
bool overlapAABB(Vec2 aMin, Vec2 aMax, Vec2 bMin, Vec2 bMax) noexcept;
bool overlapAABB(std::pair<Vec2, Vec2> a, std::pair<Vec2, Vec2> b) noexcept;
bool overlapAABB(AABB a, AABB b) noexcept;
```

Test de inclusión de cajas axis-aligned separables:

```
aMin.x <= bMax.x && bMin.x <= aMax.x && aMin.y <= bMax.y && bMin.y <= aMax.y
```

- Los **bordes que se tocan cuentan como solapamiento** (`<=`, no `<`).
- Los overloads de `pair` y de `AABB` delegan en el primero; los tres son
  `noexcept` y `[[nodiscard]]`.
- Es un broad-phase barato: filtrar pares candidatos antes del narrow-phase
  (colisión exacta). No implica colisión real de las formas, solo que sus
  envolturas se cruzan.

### 2.2 `bodyAABB`

```cpp
std::pair<Vec2, Vec2> bodyAABB(Body const& body);
```

Devuelve `{lower, upper}` en mundo vía `shapeAABB(body.shape, body.pos, body.angle)`.
Funciona para las cinco formas (círculo, `AABB`/caja, polígono convexo, cápsula,
segmento) y **tiene en cuenta la rotación** del cuerpo: una caja rotada 45°
produce una envolutura mayor que la de su caja local. Para `AABB` con ángulo
distinto de cero el resultado es la envolutura axis-aligned del polígono rotado,
no la caja original.

Uso típico:

```cpp
auto a = phys2d::bodyAABB(ballBody);
auto b = phys2d::bodyAABB(w.body(box));
bool solapan = phys2d::overlapAABB(a, b);
```

(este es exactamente el patrón usado en `examples/ex_raycast.cpp`).

---

## 3. Punto dentro de forma — `pointInShape`, `pointQuery` y primitivas

### 3.1 Primitivas geométricas

| Función | Firma (resumen) | Notas |
|---|---|---|
| `pointInCircle` | `(Vec2 p, Vec2 center, float radius) -> bool` | `noexcept`; compara `dist2 <= r²`. Borde incluido. |
| `pointInBox` | `(Vec2 p, Vec2 center, Vec2 half, float angle) -> bool` | Rota `p-center` por `-angle` al espacio local y compara con `half`. Borde incluido. |
| `pointInPolygon` | `(Vec2 p, span<Vec2 const> vertsWorld) -> bool` | Ray-casting (regla par-impar). Necesita ≥3 vértices; en mundo. `noexcept`. |
| `pointInCapsule` | `(Vec2 p, Vec2 a, Vec2 b, float radius) -> bool` | Distancia a la core-segmento `<= radius`. |

### 3.2 `pointInShape` (sobre `Body`)

```cpp
bool pointInShape(Vec2 point, Body const& body);
```

Delegación en `pointInside(body.shape, point, body.pos, body.angle)` de
`shapes.hpp`, que cubre las cinco formas. Devuelve `true` si el punto de mundo
está dentro (borde incluido, según la primitiva). No tiene estado ni asigna memoria.

### 3.3 `pointQuery` (sobre conjuntos)

```cpp
int pointQuery(Vec2 point, BodySpan bodies);
int pointQuery(Vec2 point, BodyPtrSpan bodies);
```

Devuelve el índice del **primer** cuerpo del span que contiene el punto, o `-1`
si ninguno. Implementación lineal (recorrido completo del conjunto con early-exit
en el primer hit); para mundos grandes preferir un broad-phase (hash espacial / SAP)
y luego `pointInShape` sobre los candidatos.

---

## 4. Punto más cercano — `closestPointOnShape` y primitivas

### 4.1 Primitivas geométricas

| Función | Firma (resumen) | Notas |
|---|---|---|
| `closestPointOnSegment` | `(Vec2 p, Vec2 a, Vec2 b) -> Vec2` | Proyección clamp a `t ∈ [0,1]`. Si el segmento es degenerado (`len2 <= 1e-12`) devuelve `a`. |
| `closestPointOnCircle` | `(Vec2 p, Vec2 center, float radius) -> Vec2` | Si `p` está dentro o en el borde, devuelve `p` (no proyecta hacia dentro). |
| `closestPointOnAABB` | `(Vec2 p, Vec2 aMin, Vec2 aMax) -> Vec2` | Clamp por componente. Si `p` está dentro, devuelve `p`. |
| `closestPointOnPolygon` | `(Vec2 p, span<Vec2 const> vertsWorld) -> Vec2` | Mínimo sobre todos los segmentos del borde. Vacío → `p`; un vértice → ese vértice. **Ojo:** devuelve el punto del *borde*, no del interior; si `p` está dentro del polígono, el resultado es el punto de borde más cercano, no `p`. |
| `closestPointOnCapsule` | `(Vec2 p, Vec2 a, Vec2 b, float radius) -> Vec2` | Punto más cercano del segmento-core desplazado por `radius`. Si `p` está dentro, devuelve `p`. |

Semántica uniforme: si el punto ya está dentro de la forma, el primitive
que representa un sólido devuelve `p` sin cambios; `closestPointOnPolygon`
no puede (su única información son los segmentos del borde).

### 4.2 `closestPointOnShape` (sobre `Body`)

```cpp
Vec2 closestPointOnShape(Vec2 point, Body const& body);
```

Pipeline: convierte `point` a local con `worldToLocal(point, body.pos, body.angle)`,
aplica `closestPoint(body.shape, local)` de `shapes.hpp` y vuelve a mundo con
`localToWorld`. Delega por tanto en la semántica de `shapes.hpp` (círculo,
`AABB`, polígono, cápsula, segmento). Útil para nudges de separador, grapple
points, sondas de IA y para trazar líneas entre un punto arbitrario y la superficie
de un cuerpo (ver `examples/ex_raycast.cpp`, donde se calcula desde el origen).

---

## 5. Raycast — lanzamiento de rayos

Convenciones comunes a todas las funciones de esta sección:

- `dir` sin normalizar está bien; se normaliza internamente. Un `dir` de
  longitud nula o casi nula (`<= 1e-8`) devuelve miss.
- `maxDist <= 0` devuelve miss.
- Si el origen está **dentro** de la forma, se reporta hit con
  `point = origin`, `normal = -dir`, `fraction = 0`.
- Todas son `[[nodiscard]]` y no modifican nada.

### 5.1 `raycastCircle(origin, dir, maxDist, center, radius)`

Resuelve la cuadrática del círculo: `|origin + d·t - center|² = r²` con `d`
normalizado. Descarta si el discriminante es negativo o si la raíz más cercana
está detrás del rayo o más allá de `maxDist`. Devuelve la normal radial
`(p - center).normalized()`.

```
origin=(0,0) dir=(1,0) maxDist=100 center=(50,0) r=10  →  t=40, fraction=0.40
```

### 5.2 `raycastAABB(origin, dir, maxDist, aMin, aMax)`

Método de **slabs** (losas): por cada eje se calcula el intervalo de `t` dentro
de la losa; la intersección de ambos intervalos es el trecho dentro de la caja.
Si un componente de `dir` es casi nula (`<= 1e-8`), exige que el origen ya esté
dentro de esa losa. La normal es la del eje que determinó `tMin` (con signo opuesto
al sentido de entrada).

```
origin=(0,0) dir=(1,0) maxDist=100 caja=(50,-10)-(60,10)  →  t=50, fraction=0.50, normal=(-1,0)
```

### 5.3 `raycastPolygon(origin, dir, maxDist, vertsWorld)`

Intersección de paramétricas rayo-segmento contra cada borde (`n` bordes).
Usa productos cruzados (`cross`) con tolerancia `1e-12` para descartar bordes
paralelos al rayo. Si el origen está dentro (vía `pointInPolygon`), hit en `0`.
Si no, el impacto es el borde con menor `t ∈ [0, maxDist]` y `u ∈ [0,1]`;
la normal es la perpendicular del borde orientada contra `dir`.

```
quad=(40,-10)-(60,-10)-(60,10)-(40,10)  →  t=40, fraction=0.40
```

### 5.4 `raycastSegment(origin, dir, maxDist, a, b)`

Un único par rayo-segmento (el mismo submétodo que usa `raycastPolygon` por borde).
La forma «segmento» se trata como una arista infinitamente fina: un rayo paralelo
(`|denom| <= 1e-12`) nunca impacta, aunque sea colineal. Normal = perpendicular
del segmento orientada contra `dir`.

### 5.5 `raycastCapsule(origin, dir, maxDist, a, b, radius)`

La cápsula es el barrido de un disco a lo largo del segmento `a-b`. Enfoque:

1. Si el origen está dentro (`pointInCapsule`), hit en `0`.
2. Los dos tapas se resuelven reutilizando `raycastCircle`; **solo se acepta la
   mitad exterior** de cada círculo (la interior no es borde de la cápsula):
   para el tapa `A` se exige `local <= 1e-4`, para `B`, `local >= segLen - 1e-4`.
3. Los dos lados laterales se resuelven como segmentos (`a±perp·r → b±perp·r`).

Se queda con el `t` mínimo entre las cuatro piezas. Si `a == b` (segmento
degenerado), delega en `raycastCircle`.

```
capsule=(50,-30)-(50,30) r=10, rayo horizontal  →  t=40, fraction=0.40
```

### 5.6 `raycastBody(origin, dir, maxDist, body)`

Despacha por el `variant` de `body.shape`:

| Forma | Implementación |
|---|---|
| `Circle` | `raycastCircle(origin, dir, maxDist, body.pos, s.radius)` directo. |
| `AABB` (caja) | Convierte las 4 esquinas locales a mundo (rotación `body.angle` + `body.pos`) y usa `raycastPolygon`. **Ojo:** no es slabs; el coste es 4 bordes y es correcto para cajas rotadas. |
| `ConvexPolygon` | Vértices locales rotados+trasladados a mundo (copia en `std::vector`) y `raycastPolygon`. |
| `Capsule` | Extremos `capA()/capB()` a mundo y `raycastCapsule`. |
| Segmento | `raycastSegment` si `radius <= 1e-8`, si no `raycastCapsule` (segmento con grosor). |

Es la función que `World` usa aguas abajo para cada candidato: convierte el
espacio local del cuerpo a mundo **solo para los vértices necesarios** y llama
a la primitiva correspondiente.

### 5.7 `raycastPoint` / `raycastWorld` (sobre conjuntos)

```cpp
RayHit raycastPoint(Vec2 origin, Vec2 dir, float maxDist, BodySpan bodies);
RayHit raycastPoint(Vec2 origin, Vec2 dir, float maxDist, BodyPtrSpan bodies);
RayHit raycastWorld(Vec2 origin, Vec2 dir, float maxDist, BodySpan bodies); // alias
```

Recorrido lineal: para cada cuerpo llama a `raycastBody` y conserva el hit con
menor `t`. Rellena `bodyId` con el índice del span. `raycastWorld` es alias
directo de `raycastPoint` (overload de `BodySpan`); existe para simetría
nominal con el mundo.

Sin broad-phase, el coste es `O(n)` raycast-polígono/círculo por cuerpo.
Para mundos densos se recomienda filtrar previamente por `overlapAABB` del
segmento del rayo (AABB degenerado `origin → origin+dir·maxDist`) o usar
la partición espacial de `World`.

### 5.8 Atajo de `World`

```cpp
auto hit = w.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f);          // RayHit
Vec2 p; int b;
bool ok = w.raycast({400.f, 0.f}, {0.f, 1.f}, 600.f, p, b);     // out-params
```

`World::raycast` es `const` y delega en
`raycastPoint(origin, dir, maxDist, bodiesSpan())`. Es la forma recomendada
desde código de juego: no hay que pasar spans ni preocuparse por lifetimes.
La versión con out-params es para callers que prefieren evitar el `struct`.

---

## 6. Shape cast — barrido de formas

El shape cast generaliza el raycast: en lugar de un punto, se barre una **forma**
(por defecto un círculo de cierto radio) a lo largo de `dir` y se busca el primer
instante de contacto con un objetivo estático. Convenciones idénticas a las de §5:
`dir` sin normalizar, `maxDist <= 0` o `dir ≈ 0` → miss.

### 6.1 `shapeCastCircle(start, dir, maxDist, radius, targetCenter, targetRadius)`

Equivalencia exacta con radio suma:

```
shapeCastCircle(start, dir, maxDist, r, C, R) == raycastCircle(start, dir, maxDist, C, R + r)
```

En efecto, un círculo móvil de centro `start + dir·t` y radio `r` toca el círculo
estático `(C,R)` cuando `|start + dir·t - C| = R + r`. Es el primitivo más barato
y exacto; úsenlo siempre que el barrido sea de un disco.

### 6.2 `shapeCastBody(start, dir, maxDist, radius, target)`

Barrido de un círculo de radio `radius` (centro `start`) contra un `Body` objetivo.
Despacha por la forma del objetivo, con precisión **variable** (documentada en el
código):

| Objetivo | Método | Precisión |
|---|---|---|
| `Circle` | `shapeCastCircle` (radio suma) | Exacta. |
| `Capsule` | `raycastCapsule(..., s.radius + radius)` | Exacta (cápsula engordada). |
| Segmento con grosor | `raycastCapsule(..., s.radius + radius)` | Exacta. |
| `AABB` sin rotar (`angle == 0`) | `raycastAABB` sobre la caja expandida por `radius` | **Conservadora en esquinas:** la expansión por caja sobreestima los bordes diagonales (puede reportar impacto ligeramente antes del real). |
| `AABB` rotada | `raycastPolygon` sobre las esquinas en mundo **sin expandir** | **Aproximada:** barre el centro sin engordar; subestima (puede atravesar ligeramente). |
| `ConvexPolygon` | `raycastPolygon` sin expandir | Aproximada (idem). |
| Segmento fino (`radius <= 1e-8`) | `raycastSegment` sin expandir | Aproximada. |

Regla práctica: para barrido exacto contra polígonos/cajas rotadas, usar
`shapeCastShape` (§6.3) en lugar de este helper rápido.

### 6.3 `shapeCastShape` — exacto convexo-vs-convexo (Minkowski)

```cpp
RayHit shapeCastShape(Vec2 start, Vec2 dir, float maxDist,
                      std::span<Vec2 const> movingVerts,
                      std::span<Vec2 const> targetVerts);
```

- `movingVerts`: vértices del polígono móvil **relativos a `start`**
  (en `t=0` ocupa `start + movingVerts`).
- `targetVerts`: vértices del objetivo estático en mundo.

Método de **Minkowski**: construye todas las diferencias `target - moving`
(`O(n·m)`), calcula su casco convexo con **monotone chain** (`convexHull`, §6.4) y
lanza un `raycastPolygon` desde `start` contra ese casco. El resultado es **exacto
para polígonos convexos con traslación pura** (sin rotación durante el barrido).
Si el casco tiene <3 puntos o área ≈ 0, devuelve miss. Devuelve `bodyId = -1`
(no conoce conjuntos).

```cpp
// Caja móvil 10x10 (relativa) contra caja estática
std::vector<Vec2> moving{{5,5},{5,-5},{-5,-5},{-5,5}};
std::vector<Vec2> target{{40,-10},{60,-10},{60,10},{40,10}};
auto h = phys2d::shapeCastShape({0,0}, {1,0}, 100.f, moving, target);
```

### 6.4 `convexHull`

```cpp
std::vector<Vec2> convexHull(std::span<Vec2 const> points);
```

Monotone chain: ordena por `(x,y)`, elimina duplicados exactos, construye
cadenas inferior/superior con cross-product `<= 0` (descarta puntos colineales
interiores), devuelve el casco en sentido CCW. Si tras el proceso quedan <3
puntos o el área doble (`Σ cross`) es `<= 1e-12`, devuelve los puntos ordenados
sin duplicados (sin garantizar convexidad). Expuesta públicamente porque
`shapeCastShape` la necesita y porque es útil como utilidad geométrica.

### 6.5 `shapeCastBodyShape(dir, maxDist, moving, target)`

Conveniencia sobre dos `Body`: extrae los vértices en mundo del objetivo y
relativos a `moving.pos` del móvil, y delega en `shapeCastShape(moving.pos, ...)`.
**Limitación:** solo funciona si ambos cuerpos son `AABB` o `ConvexPolygon`;
si alguna forma es círculo/cápsula/segmento devuelve miss (`hit=false`)
— para esos usar `shapeCastBody` (§6.2).

### 6.6 `shapeCastPoint(start, dir, maxDist, radius, bodies)`

Versión sobre conjuntos del barrido de círculo: recorre el span, conserva el
`shapeCastBody` con menor `t` y rellena `bodyId`. Es la equivalente de
`raycastPoint` para barridos con grosor; coste `O(n)` sin broad-phase.

### 6.7 Uso interno en `World` (anti-tunneling)

`World` usa estos helpers para CCD (continuous collision detection): los cuerpos
rápidos (`speed·h > 0`) conservan su posición al inicio del paso (`startPos_`);
si un barrido con `shapeCastBodyShape` (polígonos) o `shapeCastBody` (resto,
con `radius=0`, es decir, barrido del centro) detecta un contacto dentro del
desplazamiento, se clampa el movimiento al instante de impacto y se aplica la
reacción. Es la razón de que existan dos niveles: rápido/aproximado
(`shapeCastBody`) y exacto (`shapeCastBodyShape`).

---

## 7. DebugDraw y `drawWorld` — `debug.hpp`

### 7.1 Interfaz `DebugDraw`

```cpp
struct DebugDraw {
    virtual ~DebugDraw() = default;
    virtual void drawCircle(Vec2 center, float radius) = 0;
    virtual void drawBox(Vec2 center, Vec2 half, float angle) = 0;
    virtual void drawSegment(Vec2 a, Vec2 b) = 0;
    virtual void drawPolygon(std::span<Vec2 const> vertsWorld);  // default
    virtual void drawCapsule(Vec2 a, Vec2 b, float radius);       // default
};
```

**Métodos puros (obligatorios en el derivado):** `drawCircle`, `drawBox`,
`drawSegment`. Son el núcleo mínimo para representar cualquier forma.

**Defaults que no hay que sobreescribir** (útiles para renderers que solo
implementan primitivas):

- `drawPolygon`: itera los vértices y emite un `drawSegment` por borde
  (incluyendo el cierre `último → primero`). Si hay <2 vértices no dibuja nada.
- `drawCapsule`: emite `drawCircle(a)`, `drawCircle(b)` y las dos tangentes
  `a±perp·r → b±perp·r` como segmentos, con `perp` perpendicular al eje.

Las coordenadas son siempre **mundiales** (ya transformadas por
`drawWorld`/`drawBody`); el implementador no necesita conocer rotaciones.

### 7.2 `drawBody` y `drawWorld`

```cpp
bool drawBody(DebugDraw& dd, Body const& body);
void drawWorld(DebugDraw& dd, std::span<std::unique_ptr<Body> const> bodies);
void drawWorld(DebugDraw& dd, std::span<Body const*> bodies);
```

`drawBody` despacha por la forma:

| Forma | Llamada |
|---|---|
| `Circle` | `drawCircle(body.pos, s.radius)` |
| `AABB` | `drawBox(body.pos + s.center().rotated(angle), s.half(), body.angle)` |
| `ConvexPolygon` | `drawPolygon(vértices rotados+trasladados a mundo)` |
| `Capsule` | `drawCapsule(a en mundo, b en mundo, s.radius)` |
| Segmento | `drawSegment(a en mundo, b en mundo)` |

`drawWorld` recorre el span y llama a `drawBody` por cada cuerpo no nulo.
Acepta tanto `w.bodiesSpan()` como `span<const std::unique_ptr<Body>>` (mismo dual de §1.2).

### 7.3 Implementaciones de ejemplo (en los ejemplos)

**`TextDraw` (`examples/ex_stack.cpp`):** imprime cada llamada con `std::println`
(`drawCircle c=… r=…`, `drawBox c=… half=… a=…`, `drawSegment (x,y) -> (x,y)`).
Trazas de texto muy útiles para verificar el contenido del mundo sin gráfico.

**`SilentDraw` (`examples/ex_raycast.cpp`):** implementa los tres puros como
no-ops; `drawWorld` se invoca para probar que recorre el mundo sin fallar
(ejercicio de cobertura, no visual).

**Renderer real (para el juego):** implementar sobre SDL/ImGui/OpenGL los tres
métodos puros y delegar el resto a los defaults o sobreescribirlos con versiones
rellenadas:

```cpp
struct SDLDraw : phys2d::DebugDraw {
    SDL_Renderer* r;
    void drawCircle(phys2d::Vec2 c, float rad) override { /* SDL o círculo de líneas */ }
    void drawBox(phys2d::Vec2 c, phys2d::Vec2 half, float ang) override { /* polilínea de 4 esquinas */ }
    void drawSegment(phys2d::Vec2 a, phys2d::Vec2 b) override { /* SDL_RenderLine */ }
    // drawPolygon/drawCapsule heredan los defaults
};
SDLDraw dd;
phys2d::drawWorld(dd, w.bodiesSpan());
```

Tip: dibujar rayos (`raycast*`) y contactos con `drawSegment` (mismo canal de
depuración) para superponer queries sobre la escena. Para la cápsula, si el
renderer prefiere una única trayectoria, sobrescribir `drawCapsule` con dos
arcos y un rectángulo relleno.

---

## 8. Ejemplos

### 8.1 `examples/main.cpp` — API mínima

- Crea `World` con gravedad `(0, 900)`, suelo estático, dos círculos dinámicos
  y una caja rotada (`angle = 0.3f`).
- Simula 180 pasos a 60 Hz e imprime `pos/vel/angle` de cada cuerpo (recorrido
  con `for (auto& body : world)`).
- Dispara `w.raycast({400,0}, {0,1}, 600.f)` hacia abajo e imprime
  `hit/body/frac` de la primera intersección.
- Muestra el estilo de inicializadores designados completos de C++23
  (no se mezcla `{{100,50}, .radius=20}`).

```
inicio: 4 cuerpos
pos=... vel=... angle=...
contactos: ...
raycast down hit=... body=... frac=...
```

### 8.2 `examples/ex_raycast.cpp` — batería de queries

Setup: suelo estático, bola (círculo r=20), caja rotada 0.3 rad, cápsula y
triángulo (polígono). Simula 120 pasos y luego:

1. **`w.raycast` hacia abajo** desde `(400,0)`: impacto contra la columna apilada.
   Imprime `hit/body/frac/point/normal`.
2. **`raycastPoint` lateral** desde `(0,50)` con `w.bodiesSpan()`: demostración de la
   vía sin `World` (mismo resultado sin pasar por el atajo).
3. **Primitivas con expectativas cerradas** (respaldos de regresión):
   ```cpp
   raycastCircle(origin=0, dir=(1,0), max=100, center=(50,0), r=10)   → frac 0.40
   raycastAABB(origin=0, dir=(1,0), max=100, (50,-10)-(60,10))        → frac 0.50
   raycastPolygon(quad 40..60)                                        → frac 0.40
   raycastCapsule((50,-30)-(50,30), r=10)                             → frac 0.40
   ```
4. **`pointInShape(centro de la bola)`** → `true`; **`closestPointOnShape(origen)`**
   → punto del borde más cercano al origen.
5. **`bodyAABB` + `overlapAABB`** entre bola y caja (solapamiento de envolturas).
6. **`drawWorld` con `SilentDraw`** sobre `w.bodiesSpan()` y chequeo de
   `w.bodiesSpan().size()` (cobertura del dual de spans).

### 8.3 `examples/ex_stack.cpp` — pila, contactos y debug

- Cuatro cajas dinámicas apiladas sobre el suelo (desviación inicial en `x` de
  1 px por nivel para romper simetría), 360 pasos a 60 Hz con `onBeginContact`
  imprimiendo `a/b/depth`.
- Verifica estabilidad numérica: desviación vertical máxima respecto al
  `expectY` teórico; imprime `PASS` si `maxDesviacionY <= 2.f`, si no `WARN>2px`.
- **`drawWorld` con `TextDraw`** (trazas legibles de cada forma).
- **`raycastPoint` desde `(400,0)` hacia abajo** contra `w.bodiesSpan()`: conserva el
  `point.y` del primer impacto (la pila).
- Recorrido `for (auto& b : w)` imprimiendo posiciones.

### 8.4 `examples/ex_chain.cpp` — cadena con joints

- Ancla estática y 8 círculos dinámicos unidos por joints `Distance` con
  `rest = 30.f` (cadena colgante 8×30 = 240 px).
- Simula 240 pasos y muestra `pos/vel` por eslabón y la posición de la cola
  (longitud total ~`8·rest`).
- No usa queries directamente; es el ejemplo de fondo contra el que se
  combinan `pointInShape`/`closestPointOnShape` (grapple sobre eslabones)
  o `raycast*` (cortar la cadena con un rayo).

### 8.5 Benchmark — `examples/bench.cpp` (+ binario `bench/`)

Banco de rendimiento del solver, no de queries, pero incluido como parte del
paquete de ejemplos asociado: mide el coste de `World::step` bajo carga.

**Argumentos** (posicionales, parseados con `std::from_chars`; valores inválidos
o `<= 0` caen al fallback):

```bash
./bench [N] [steps] [cellSize] [broadphase]
```

| Pos | Param | Default | Significado |
|---|---|---|---|
| 1 | `N` | `10000` | Nº de círculos dinámicos. |
| 2 | `steps` | `120` | Nº de pasos de `1/60 s`. |
| 3 | `cellSize` | `64` | Tamaño de celda del spatial hash. |
| 4 | `broadphase` | `sh` | `sap` → sweep-and-prune; cualquier otro → spatial hash. |

**Escena:** gravedad `(0,900)`, suelo estático en `(1100,590)` half `(2000,10)`
y `N` círculos de radio `8` dispuestos en retícula de 100 columnas con paso
`20` px desde `(100,50)` (todas dinámicas, sin joints).

**Salida** (una línea `std::println`):

```
bench N=10000 steps=120 cellSize=64 broadphase=sh total=123.45ms per-step=1.029ms contacts=... bodies=10001
```

Mide solo el bucle `world.step(1/60)` con `steady_clock` (`t0`/`t1`), e imprime
además el conteo de contactos y cuerpos finales. Usos sugeridos:

```bash
./bench 10000 120 64 sh      # baseline por defecto
./bench 10000 120 64 sap     # compara sweep-and-prune
./bench 20000 240 32 sh      # estrés: más cuerpos y celdas pequeñas
```

---

## 9. Tabla de referencia rápida

| Consulta | Entrada | Devuelve | Coste |
|---|---|---|---|
| `raycastCircle` | origen, dir, maxDist, centro, r | `RayHit` | O(1) |
| `raycastAABB` | origen, dir, maxDist, aMin, aMax | `RayHit` | O(1) (slabs) |
| `raycastSegment` | origen, dir, maxDist, a, b | `RayHit` | O(1) |
| `raycastPolygon` | origen, dir, maxDist, vertsWorld | `RayHit` | O(n) bordes |
| `raycastCapsule` | origen, dir, maxDist, a, b, r | `RayHit` | O(1) (4 piezas) |
| `raycastBody` | origen, dir, maxDist, body | `RayHit` | O(v) vértices |
| `raycastPoint`/`raycastWorld` | origen, dir, maxDist, span | `RayHit` (+bodyId) | O(n·v) |
| `World::raycast` | origen, dir, maxDist | `RayHit` (+bodyId) | O(n·v) |
| `shapeCastCircle` | start, dir, maxDist, r, centro, R | `RayHit` | O(1), exacta |
| `shapeCastBody` | start, dir, maxDist, r, target | `RayHit` | O(v), mixta |
| `shapeCastShape` | start, dir, maxDist, moving, target | `RayHit` | O(n·m + h log h) |
| `shapeCastBodyShape` | dir, maxDist, moving, target | `RayHit` | O(n·m + h log h), polígonos |
| `shapeCastPoint` | start, dir, maxDist, r, span | `RayHit` (+bodyId) | O(n·v) |
| `pointInShape` | point, body | `bool` | O(1)/O(v) |
| `pointQuery` | point, span | `int` índice o -1 | O(n·v) |
| `closestPointOnShape` | point, body | `Vec2` mundo | O(1)/O(v) |
| `overlapAABB` | dos AABB/pairs | `bool` | O(1) |
| `bodyAABB` | body | `pair<Vec2,Vec2>` | O(v) |
| `drawBody`/`drawWorld` | `DebugDraw&`, body/span | `bool`/`void` | O(n·v) dibujo |

---

## 10. Recetas

**Picking de ratón (primer cuerpo bajo el cursor):**

```cpp
auto hit = phys2d::raycastPoint(phys2d::Vec2{}, {0, 0}, 0, w.bodiesSpan()); // placeholder
int id = phys2d::pointQuery(mouseWorld, w.bodiesSpan());
if (id >= 0) { /* ... w.body(id) ... */ }
```

**Línea de visión (segmento bloqueado):**

```cpp
phys2d::Vec2 d = target - eye;
float dist = d.length();
auto hit = w.raycast(eye, d, dist);
bool visible = !hit.hit || hit.fraction >= 0.999f;
```

**Salto con control de aterrizaje (barrido de pies):**

```cpp
auto hit = phys2d::shapeCastPoint(feet, {0, 1}, maxFall, bodyRadius, w.bodiesSpan());
if (hit.hit) { /* aterrizar en hit.point */ }
```

**Superponer la query de depuración en el renderer:**

```cpp
if (hit.hit) dd.drawSegment(eye, hit.point);  // reutiliza DebugDraw
```
