# Joints / Constraints — phys2d

> Documentación extensa en español de los enlaces (joints) y solvers sueltos de phys2d.
> Fuentes: `include/phys2d/joint.hpp`, `include/phys2d/constraints.hpp`,
> `include/phys2d/world.hpp` (helpers `addJoint` / `addXJoint`).

## 1. Visión general

Un **joint** es una restricción que une dos cuerpos `a` y `b` manteniendo una
relación geométrica (distancia, ángulo, eje, longitud total, fase angular...).
A diferencia de un contacto —que solo aparece cuando dos formas se tocan—,
el joint **siempre existe** en el `World` y se resuelve en cada `step(dt)`,
junto con los contactos, mediante impulsos secuenciales con corrección de
posición (Baumgarte: `beta = 0.2`, `gamma = 0` por defecto).

Arquitectura en tres capas:

| Capa | Archivo | Contenido |
|---|---|---|
| Base | `joint.hpp` | `enum JointType`, `Joint` abstracta, `pointVelocity`, `applyImpulse`, `applyAngularImpulse`, y las 9 clases de joint: Distance, Revolute, Prismatic, Pulley, Spring, Weld, Gear, Rope, Slider |
| Solvers sueltos | `constraints.hpp` | funciones libres `solveDistance`, `solveRevolute`, `solveSpring`, `solvePrismatic`, `solveWeld`, `solveRope`, `solveGear`, `solvePulley` que delegan en las clases de `joint.hpp` |
| Integración | `world.hpp` | `World::jointCount()`, `World::joint(id)`, `addJoint(unique_ptr<Joint>)`, plantilla `addJoint<JointT>(args...)` y helpers `addDistanceJoint`, `addRevoluteJoint`, `addWeldJoint`, `addPrismaticJoint`, `addSliderJoint`, `addPulleyJoint`, `addSpringJoint`, `addRopeJoint`, `addGearJoint` |

`phys2d.hpp` incluye `constraints -> joint -> ... -> world`, por lo que con
`#include "phys2d/phys2d.hpp"` tienes todo disponible.

### 1.1 `JointType`

```cpp
enum class JointType {
    Distance, Revolute, Prismatic, Pulley, Spring, Weld, Gear, Rope, Slider
};
```

Cada clase derivada fija `type` en su constructor. El método
`bool isType(JointType) const noexcept` permite discriminar sin `dynamic_cast`.

### 1.2 `Joint` (clase base abstracta)

```cpp
struct Joint {
    int a = -1, b = -1;          // ids de los cuerpos en World
    JointType type = JointType::Distance;
    float maxImpulse = 1.0e30f;  // saturación del impulso acumulado
    float beta = 0.2f;           // corrección de violación posicional (Baumgarte)
    float gamma = 0.f;           // regularización / softness (Spring la usa)
    Vec2 bias{};                 // sesgo vectorial de velocidad (posición)
    float biasScalar = 0.f;      // sesgo escalar (1D)
    float biasAngle = 0.f;       // sesgo angular
    float mass = 0.f;            // masa efectiva (1 / K) — se calcula en prepare
    float massAngular = 0.f;     // masa efectiva angular
    Vec2 impulse{};              // impulso acumulado (2D)
    float impulseScalar = 0.f;   // impulso acumulado (1D)
    float angularImpulse = 0.f;  // impulso angular acumulado
    float motorImpulse = 0.f;    // impulso del motor (Revolute)
    Vec2 ra{}, rb{};             // brazos de palanca mundo: puntoAncla - pos
    Vec2 unit{};                 // dirección unitaria cacheada

    virtual std::unique_ptr<Joint> clone() const = 0;
    virtual void prepare(const Body& bodyA, const Body& bodyB, float invDt) = 0;
    virtual void solve(const Body& bodyA, Body& bodyAmut,
                       const Body& bodyB, Body& bodyBmut) = 0;
    void solve(Body& bodyA, Body& bodyB);  // overload sin const
};
```

Convenciones importantes:

* **Anclas (anchors)** en coordenadas **locales**: el punto mundial se obtiene
  con `body.worldPoint(anchorLocal)`. `ra = worldA - bodyA.pos`,
  `rb = worldB - bodyB.pos` se cachean en `prepare`.
* **`prepare(bodyA, bodyB, invDt)`** se llama una vez por substep antes del
  bucle de iteraciones: calcula `ra`, `rb`, `unit`, masa efectiva, sesgos
  (`bias`, `biasScalar`, `biasAngle`) y **resetea los impulsos acumulados**.
* **`solve`** se llama `velocityIterations` veces: aplica el impulso
  `Δ = clamp(impulse + λ) - impulse`, donde `λ = -mass * (v + bias + gamma*impulse)`.
* `mass <= 0` significa "sin restricción activa": `solve` sale sin tocar nada.
  Ocurre con cuerpos con `invMass = invInertia = 0` (estáticos/kinemáticos
  entre sí) o con Rope inactiva.

### 1.3 Utilidades libres

```cpp
Vec2 pointVelocity(const Body& b, Vec2 arm) noexcept;        // v + r × ω
void applyImpulse(Body& b, Vec2 imp, Vec2 arm) noexcept;     // v += J·invMass, ω += invInertia·(r×J)
void applyAngularImpulse(Body& b, float imp) noexcept;       // ω += invInertia·J
```

`r.perpendicular()` produce el resultado de `ω × r` en 2D. Todas las clases de
joint usan estas funciones: mantener torque y velocidad lineal coherentes depende de
usarlas siempre en vez de tocar `vel`/`angVel` a mano.

### 1.4 Parámetros compartidos: `beta`, `gamma`, `maxImpulse`

* **`beta`** (0.2): cuánto de la penetración posicional se corrige **por
  iteración** como velocidad sesgada (`bias = beta·invDt·error`). Valores
  mayores (0.5–0.8) corrigen más rápido pero pueden temblar; valores menores
  hacen el joint más elástico sin serlo.
* **`gamma`** (0): factor de regularización `v_target + gamma·impulse`. Se usa
  para ablandar una restricción con acumulación de impulso; en esta versión
  `Spring` lo fija en `0` (amortiguación explícita) y en el resto
  está disponible como perilla de "softness".
* **`maxImpulse`** (1e30): tope del impulso acumulado, para romper el joint
  bajo carga o limitar la fuerza transmitida. Es un *clamp de impulso*, no una
  ruptura con evento — no hay callback de rotura.

---

## 2. Distance

**Qué mantiene:** la distancia entre dos anclas igual a `restLength`, en ambas
direcciones (tira y empuja).

**Clase:**

- `joint.hpp`: `struct Distance : Joint`
- Constructores:
  - `Distance(int idA, int idB, float restLen)` — anclas en el centro.
  - `Distance(int idA, int idB, float restLen, Vec2 anchorALocal, Vec2 anchorBLocal)`.

**Campos propios:**

| Campo | Significado |
|---|---|
| `restLength` | Distancia objetivo entre anclas |
| `stiffness` (1.0) | Multiplica la masa efectiva; `< 1` ablanda |
| `damping` (0) | Añade amortiguación: `biasScalar += damping·v_rel·dt·effK` |
| `anchorA`, `anchorB` | Anclas locales |
| `distCached` | Distancia actual en el `prepare` |

**Matemática (`prepare`):** `unit = Δ/len`, `effK = invMassA + invMassB +
invInertiaA·(ra×unit)² + invInertiaB·(rb×unit)²`, `mass = 1/effK`,
`biasScalar = beta·invDt·(len - restLength)`.
Con `damping > 0` se suma `biasScalar += damping·relVel·dt·effK`
(amortiguación explícita: el impulso `damping·v` se integra en `dt` y se
convierte en velocidad relativa vía `effK`).
`solve`: restricción 1D sobre la velocidad relativa proyectada en `unit`,
impulso clampado a `±maxImpulse`, aplicado como `(-applied, +applied)`.

**Helpers:**

```cpp
world.addDistanceJoint(a, b, rest, anchorA, anchorB);   // en world.hpp
world.addJoint<Distance>(a, b, rest);                   // plantilla
```

**Cuándo usarlo:** péndulos isócronos, cadenas, barras articuladas
bidireccionales, péndulo con `Revolute` imposible de expresar en 1D...

> **Nota:** Distance permite que el cuerpo rote libremente alrededor del ancla;
> no fija orientación. Si necesitas "un péndulo" con barra rígida visible, usa
> Revolute ( §3 ) o Weld ( §7 ) para la barra, Distance para la cuerda.

---

## 3. Revolute

**Qué mantiene:** dos anclas **coincidentes** (2 restricciones de velocidad 2D):
el cuerpo `b` puede rotar alrededor del punto común, pero no trasladarse.
Opcionalmente, un **motor** angular (restricción de velocidad pura).

**Clase:** `struct Revolute : Joint`

- Constructor: `Revolute(int idA, int idB, Vec2 anchorALocal, Vec2 anchorBLocal)`.

**Campos propios:**

| Campo | Significado |
|---|---|
| `anchorA`, `anchorB` | Anclas locales (deben coincidir en el mundo) |
| `maxMotorTorque` | Tope del impulso del motor (1e30 = sin límite) |
| `enableMotor`, `motorSpeed` | Motor: `motorSpeed` es velocidad angular deseada (rad/s) |
| `k11`, `k12`, `k22`, `invDet`, `kInvC0`, `kInvC1` | Matriz de masa 2×2 invertida en `prepare` |

**Matemática:** construye la matriz de masa efectiva 2×2
`K = [[k11,k12],[k12,k22]]` de ambas anclas y la invierte
(`invDet = 1/(k11·k22 - k12²)`; si `|det| < 1e-12`, `invDet = 0` y `solve`
ignora la parte lineal). `bias = gapVec·beta·invDt` fuerza la coincidencia.
En `solve`, el impulso 2D se calcula con la versión inversa
`KInvC0/KInvC1` y se clampa por **módulo** a `maxImpulse`
(`len > maxImpulse ? maxImpulse/len : 1`). A continuación, si
`enableMotor`, resuelve la restricción angular 1D
`ωB - ωA = motorSpeed` con su propio `motorImpulse` clampado a `maxMotorTorque`.

**`massAngular`:** `1/(invInertiaA + invInertiaB)` — se usa solo para el motor.

**Helpers:**

```cpp
world.addRevoluteJoint(a, b, anchorA, anchorB);
world.addJoint<Revolute>(a, b, anchorA, anchorB);
```

**Casos típicos:** bisagras, rueda y eje, péndulo rígido, brazo robótico,
puerta sobre marco. Para un péndulo simple unido a un ancla estática,
`addRevoluteJoint(anchor, bob, {0,0}, {0,-halfLen})` es suficiente.

**Motor:** parámetros se ajustan accediendo al joint tras crearlo:

```cpp
int j = world.addRevoluteJoint(a, b, {0,0}, {0,0});
auto& rev = static_cast<phys2d::Revolute&>(world.joint(j));
rev->enableMotor = true;
rev->motorSpeed = 2.0f;        // rad/s
rev->maxMotorTorque = 500.f;   // tope del par
```

---

## 4. Prismatic

**Qué mantiene:** el movimiento relativo restringido a un **eje** (deslizamiento):
sin desplazamiento perpendicular, con despliegue axial limitado entre
`lower` y `upper` (topes).

**Clase:** `struct Prismatic : Joint`

- Constructor: `Prismatic(int idA, int idB, Vec2 axis, float low, float high)`.
- En `prepare` se normaliza en un miembro interno `axisNorm` (el campo `axis`
  del joint se conserva tal cual lo escribió el usuario; si `|axis| < 1e-8` se
  usa `{1,0}`).
- `perp = axisNorm.perpendicular()` se cachea.

**Campos propios:**

| Campo | Significado |
|---|---|
| `axis` | Dirección del deslizamiento (ver `axisNorm` en `prepare`; no se muta) |
| `lower`, `upper` | Topes de desplazamiento axial (−1e30 / +1e30 = sin tope) |
| `anchorA`, `anchorB` | Anclas; definen `ra`/`rb` y el desplazamiento axial |
| `enableLimit` (true) | Si `false`, los topes se ignoran |
| `perp`, `massPerp`, `massAxis` | Perpendicular y masas efectivas de cada dirección |
| `biasPerp`, `biasAxis` | Sesgos perpendicular (siempre) y axial (solo fuera de rango) |
| `dispCached` | Desplazamiento axial actual `(worldB-worldA)·axis` |
| `limitImpulse` | Impulso acumulado **de los topes** (independiente del perpendicular) |

**Matemática — dos restricciones:**

1. **Perpendicular (siempre):** `v_rel·perp + biasPerp + gamma·impulseScalar`
   con `massPerp = 1/K_perp`. Es lo que impide el desplazamiento lateral.
2. **Axial de tope (solo si `dispCached` está fuera de `[lower, upper]`):**
   `v_rel·axis + biasAxis` con `massAxis`, clampado **unidireccionalmente**:
   si excede `lower`, el impulso solo puede empujar hacia `+axis`
   (`newLim = max(newLim, 0)`); si excede `upper`, solo hacia `−axis`
   (`min(newLim, 0)`). Es decir, los topes son **unilaterales**: no atrapan,
   solo detienen el escape. Dentro del rango no hay restricción axial:
   el cuerpo se desliza libremente.

**Helper:**

```cpp
world.addPrismaticJoint(a, b, axis, lower, upper, anchorA = {}, anchorB = {});
world.addJoint<Prismatic>(a, b, axis, lower, upper);
```

**Casos típicos:** pistón, compuerta, suspensión telescópica, puerta
corredera, jeringa/actuador lineal.

**Slider** ( §9 ) es un wrapper que delega en un `Prismatic` interno:
misma matemática con nombre distinto.

---

## 5. Pulley

**Qué mantiene:** la **longitud total** de una cuerda que pasa sobre dos
poleas: `lenA + ratio·lenB = restTotal`.

**Clase:** `struct Pulley : Joint`

- Constructor:
  `Pulley(int idA, int idB, Vec2 anchorALocal, Vec2 anchorBLocal,
          Vec2 groundAWorld, Vec2 groundBWorld, float ratioVal)`.

**Campos propios:**

| Campo | Significado |
|---|---|
| `anchorA`, `anchorB` | Anclas locales en cada cuerpo |
| `groundA`, `groundB` | Posición **mundial** de cada polea (no son cuerpos) |
| `ratio` | Multiplicador de la longitud del lado B (p. ej. polea reductor) |
| `totalLength` | `lenA + ratio·lenB` actual |
| `restTotal` | Longitud total objetivo |
| `restTotalSet` | `false` por defecto: en el primer `prepare` se congela `restTotal = totalLength` y se pone a `true`. Usa `true` para objetivos explícitos |
| `unitA`, `unitB` | Direcciones de cada tramo |

**Matemática:** la restricción es 1D:
`vA·unitA + ratio·vB·unitB + biasScalar = 0`, con
`biasScalar = beta·invDt·(totalLength - restTotal)`.
La masa efectiva incorpora el `ratio²` en el término de `B`
(`effK = invMassA + ratio²·invMassB + invInertiaA·cA² + ratio²·invInertiaB·cB²`).
El gradiente completo es `(unitA, ratio·unitB)`, y el impulso generalizado
se aplica **con el mismo signo en ambos cuerpos**:
`A += unitA·Δ`, `B += ratio·unitB·Δ` (diferencia clave frente a Distance /
Spring, donde el signo se invierte).

**Auto-congelamiento**: `Pulley` nace con `restTotalSet = false`: la primera
llamada a `prepare` congela `restTotal = totalLength` y marca el flag. Si
quieres una longitud objetivo concreta, asigna `restTotal` **y**
`restTotalSet = true` antes de `prepare` (en `World` se rehace cada step; con
el solver suelto debes fijarlos explícitamente — ver §10).

**Helper:**

```cpp
world.addPulleyJoint(a, b, anchorA, anchorB, groundA, groundB, ratio);
world.addJoint<Pulley>(a, b, anchorA, anchorB, groundA, groundB, ratio);
```

**Casos típicos:** polea fija + masa colgante, sistema de contrapeso,
polipasto (ratio ≠ 1), ascensor con dos cabinas.

---

## 6. Spring

**Qué mantiene:** una **fuerza elástica** que tira de B hacia la longitud
de reposo, con rigidez `stiffness` (N/m) y amortiguación `damping` (N·s/m).
A diferencia de Distance, no es una restricción rígida: es un **resorte
blando** integrado explícitamente en el bucle de velocidades (ver la nota de
esta sección).

**Clase:** `struct Spring : Joint`

- Constructor: `Spring(int idA, int idB, float restLen, float stiff, float damp)`.
  Los anclas se fijan después (`anchorA`/`anchorB`), con el helper.

**Campos propios:**

| Campo | Significado |
|---|---|
| `stiffness` (100) | Rigidez elástica K |
| `damping` (0) | Amortiguación C |
| `restLength` | Longitud de reposo |
| `anchorA`, `anchorB` | Anclas locales |
| `distCached` | Distancia actual |

**Matemática (`prepare`):**

```text
dt    = 1/invDt (o 1/60 si invDt ≈ 0)
mass  = 1/effK                            // como en Distance
gamma = 0
biasScalar = (stiffness·gapC + damping·v_rel·unit) · dt · effK
```

`effK` y `effMass` son los del caso rígido. `biasScalar` es la velocidad
relativa que el muelle querría impartir en un `dt` (fuerza × `dt` × `effK` =
Δv relativa), de modo que `solve` es el de Distance con ese sesgo: cuanto
mayor `stiffness·dt²` respecto a la masa, más rápido y más inestable; la
condición práctica es `stiffness·dt² << effMass`. Para resortes muy rígidos
baja `stiffness`, sube `damping` o los substeps.

**Claves prácticas:**

* Con `stiffness` muy alta y `dt` grande, el resorte puede oscilar o
  inestabilizarse: baja `stiffness`, sube `damping` o sube substeps.
* `damping` alto amortigua la oscilación (útil en amortiguadores).
* El resorte **tira** y **empuja** (si está comprimido bajo `restLength`,
  empuja). Si solo quieres tirar, usa Rope ( §8 ).
* En el test de la suite, un `Spring(anchor, b, 50, 100, 1)` lleva un cuerpo
  de `x=200` a `x=150` (restLength = 50) en 10 s sin gravedad.

**Helper:**

```cpp
// Spring no tiene constructor con anclas: el helper los asigna tras construirlo.
world.addSpringJoint(a, b, rest, stiffness, damping, anchorA = {}, anchorB = {});
world.addJoint<Spring>(a, b, rest, stiffness, damping);
```

---

## 7. Weld

**Qué mantiene:** todo lo que mantiene **Revolute** (anclas coincidentes) **y
además** el ángulo relativo: los dos cuerpos quedan **soldados como un solo
rígido**.

**Clase:** `struct Weld : Joint`

- Constructor: `Weld(int idA, int idB, Vec2 anchorALocal, Vec2 anchorBLocal)`.

**Campos propios:**

| Campo | Significado |
|---|---|
| `anchorA`, `anchorB` | Anclas locales coincidentes |
| `referenceAngle` | Ángulo relativo objetivo (Δθ entre cuerpos) |
| `k11`, `k12`, `k22`, `invDet`, `kInvC0`, `kInvC1` | Matriz 2×2 invertida (igual que Revolute) |

**Matemática:** `prepare` es el mismo 2×2 que Revolute más la parte angular:
`massAngular = 1/(invInertiaA + invInertiaB)`,
`biasAngle = beta·invDt·(θB - θA - referenceAngle)`.
`solve`: primero la parte 2D idéntica (impulso clampado por módulo), luego
la restricción angular 1D `ωB - ωA + biasAngle + gamma·angularImpulse = 0`
con `angularImpulse` clampado a `±maxImpulse`. **No hay motor** (eso es
Revolute); la diferencia es que aquí el motor sería el propio sesgo angular.

**Helper:**

```cpp
world.addWeldJoint(a, b, anchorA, anchorB);
world.addJoint<Weld>(a, b, anchorA, anchorB);
```

**Casos típicos:** vehículos de piezas (chasis + ruedas fijas), estructuras
compuestas, ragdolls parciales, plataformas con piezas soldadas.

**Teorema de equivalencia:** `Weld = Revolute` con el ángulo
*bloqueado* por sesgo angular (`referenceAngle`) — el matiz: Weld clampa
`angularImpulse` a `maxImpulse` en vez de `maxMotorTorque`. `Revolute`
está libre de sesgo angular: para fijar el ángulo relativo usa `Weld`.

---

## 8. Rope

**Qué mantiene:** la distancia **máxima** entre dos anclas: la cuerda solo
actúa cuando `len > maxLength`. Por debajo, no hace nada (holgura).

**Clase:** `struct Rope : Joint`

- Constructor: `Rope(int idA, int idB, float maxLen)`.
- `anchorA`/`anchorB` se asignan después (el solver suelto usa centros).

**Campos propios:**

| Campo | Significado |
|---|---|
| `maxLength` (10) | Longitud máxima permitida |
| `anchorA`, `anchorB` | Anclas locales |
| `active` | `true` solo cuando `len > maxLength` en el último `prepare` |
| `distCached` | Distancia actual |

**Matemática:** si no está activa, `mass = 0` y `solve` sale inmediatamente.
Activa: misma masa efectiva y sesgo que Distance (`beta·invDt·(len - maxLength)`),
pero con **clamp unilateral**: el impulso acumulado solo puede ser `≤ 0`
(`newImpulse = clamp(impulse+λ, -maxImpulse, 0)`). Es decir, la cuerda **solo
tira**: nunca empuja para separar los cuerpos cuando están cerca.

**Helper:**

```cpp
world.addRopeJoint(a, b, maxLength);
world.addJoint<Rope>(a, b, maxLength);
```

> **Rope ≠ Distance:** Rope admite holgura (sin efecto hasta `maxLength`) y
> nunca empuja. Distance mantiene la distancia exacta en ambos sentidos.
> Rope ≠ Spring: Rope es rígida cuando está tensa; Spring siempre ejerce
> fuerza proporcional al desplazamiento desde `restLength`.

---

## 9. Gear

**Qué mantiene:** una relación **angular** entre los dos cuerpos:
`θB + ratio·θA = phase` (los engranajes rotan en sentidos opuestos si el
ratio es negativo, mismo sentido si es positivo).

**Clase:** `struct Gear : Joint`

- Constructor: `Gear(int idA, int idB, float ratioVal)`.
- No tiene anclas: trabaja solo con ángulos.

**Campos propios:**

| Campo | Significado |
|---|---|
| `ratio` | Relación de transmisión (p. ej. `-2`: B gira al doble en sentido contrario) |
| `phase` | Fase: valor objetivo de `θB + ratio·θA` |

**Matemática:** solo restricción angular 1D.

```text
effK        = invInertiaB + ratio²·invInertiaA
massAngular = 1/effK
biasAngle   = beta·invDt·(θB + ratio·θA - phase)
solve: v = ωB + ratio·ωA + biasAngle + gamma·angularImpulse
       λ = -massAngular·v, clamp angularImpulse a ±maxImpulse
       A += ratio·λ (por el gradiente), B += λ
```

Comprobación con la suite de tests: `Gear(a, b, -2)` con `ωA₀ = 3` produce
`ωB + (-2)·ωA ≈ 0` de forma estable.

**Helper:**

```cpp
world.addGearJoint(a, b, ratio);
world.addJoint<Gear>(a, b, ratio);
```

> **Gear no genera desplazamiento ni contacto:** los engranajes se intersectan
> visualmente si sus formas se solapan; el joint solo impone la relación de
> velocidades/ángulos. El par motor y la fricción de los ejes los pones tú
> con fuerzas externas o motores Revolute.

---

## 10. Slider

**Qué es:** un `Prismatic` con otro nombre. Internamente contiene un
`Prismatic inner` y delega `prepare`/`solve` en él. Misma física, mismo
significado de campos.

**Clase:** `struct Slider : Joint`

- Constructor: `Slider(int idA, int idB, Vec2 axis, float low, float high)`.
  En el constructor copia los parámetros al `inner` (`a`, `b`, `type`,
  `axis`, `lower`, `upper`, `maxImpulse`, `beta`, `gamma`).
- `prepare` sincroniza `anchorA`/`anchorB`/`maxImpulse`/`beta`/`gamma` en
  `inner` antes de delegar, y copia de vuelta `ra`, `rb`, `bias`,
  `biasScalar`, `mass`, `massAngular`, `biasAngle`, `impulse`, `impulseScalar`,
  `angularImpulse`.
- `solve` delega en `inner.solve` y copia `impulse`/`impulseScalar`.

**Campos propios:** `axis`, `lower`, `upper`, `anchorA`, `anchorB`,
`Prismatic inner`.

**Helper:**

```cpp
world.addSliderJoint(a, b, axis, lower, upper, anchorA = {}, anchorB = {});
world.addJoint<Slider>(a, b, axis, lower, upper);
```

**Cuándo elegir Slider vs Prismatic:** son equivalentes; Slider es más
legible cuando el mecanismo es una "guía deslizante" (puerta corredera,
soporte de suspensión) y Prismatic cuando es un "vástago/pistón".
Si alguna vez divergen, `Slider::inner` te deja ajustar el Prismatic
explícitamente.

---

## 11. Helpers `addJoint` / `addXJoint` en `World`

### 11.1 `addJoint` base

```cpp
int addJoint(std::unique_ptr<Joint> joint);  // lanza std::invalid_argument si es nulo
```

```cpp
world.addJoint(std::make_unique<phys2d::Distance>(a, b, 30.f));  // base
```

Devuelve el **índice** del joint (útil para acceder luego vía `joint(id)` —
ver §11.4). `addJoint` valida los ids de cuerpos: `std::out_of_range` si
están fuera de rango, `std::invalid_argument` si `a == b` o el puntero es nulo.

### 11.2 Plantilla `addJoint<JointT>(args...)`

```cpp
template <typename JointT, typename... Args>
int addJoint(Args&&... args);
```

```cpp
world.addJoint<phys2d::Distance>(a, b, 30.f);
world.addJoint<phys2d::Spring>(a, b, 50.f, 100.f, 1.f);
```

**Limitación conocida:** la plantilla variádica no deduce `Vec2` desde
braced-init. Por eso `addRevoluteJoint(a, b, {0,-100}, {0,0})` funciona
directamente con `{0,0}` a pelo, pero
`addJoint<Revolute>(a, b, {0,-100}, {0,0})` **no** compila: hay que escribir
`phys2d::Vec2{0,-100}`. Los helpers `addXJoint` existen precisamente para
esto.

### 11.3 Helpers concretos (`AddXJoint`)

| Helper | Firma | Asignación |
|---|---|---|
| `addDistanceJoint` | `int (int a, int b, float rest, Vec2 anchorA = {}, Vec2 anchorB = {})` | `addJoint<Distance>(a,b,rest,anchorA,anchorB)` |
| `addRevoluteJoint` | `int (int a, int b, Vec2 anchorA, Vec2 anchorB)` | `addJoint<Revolute>(...)` |
| `addWeldJoint` | `int (int a, int b, Vec2 anchorA, Vec2 anchorB)` | `addJoint<Weld>(...)` |
| `addPrismaticJoint` | `int (int a, int b, Vec2 axis, float lower, float upper, Vec2 anchorA = {}, Vec2 anchorB = {})` | construye `Prismatic`, asigna anclas, `addJoint(move)` |
| `addSliderJoint` | idem | construye `Slider`, asigna anclas, `addJoint(move)` |
| `addPulleyJoint` | `int (int a, int b, Vec2 anchorA, Vec2 anchorB, Vec2 groundA, Vec2 groundB, float ratio)` | `addJoint<Pulley>(...)` |
| `addSpringJoint` | `int (int a, int b, float rest, float stiffness, float damping, Vec2 anchorA = {}, Vec2 anchorB = {})` | construye `Spring`, asigna anclas, `addJoint(move)` |
| `addRopeJoint` | `int (int a, int b, float maxLength)` | `addJoint<Rope>(...)` |
| `addGearJoint` | `int (int a, int b, float ratio)` | `addJoint<Gear>(...)` |

Devuelven el índice del joint. Ejemplo del test
`testJointHelpersAcceptBraces` (tests.cpp):

```cpp
// Sin phys2d::Vec2{} explícito: los helpers aceptan {0,0} a pelo.
(void)world.addRevoluteJoint(a, b, {0.f, 0.f}, {0.f, -100.f});
```

### 11.4 Acceso y consulta

```cpp
float const count = static_cast<float>(world.jointCount());  // nº de joints vivos
Joint const& joint = world.joint(id);                        // lanza std::out_of_range
joint.a; joint.b; joint.type; joint.isType(JointType::Revolute);
joint.impulseScalar; joint.impulse; joint.angularImpulse;    // para debug
```

Si necesitas modificar un parámetro tras crearlo (p. ej. `stiffness` o
`maxImpulse`), usa el índice devuelto y `static_cast` al tipo concreto:

```cpp
int idx = world.addRevoluteJoint(a, b, {0,0}, {0,-100});
auto& rev = static_cast<phys2d::Revolute&>(world.joint(idx));
rev.enableMotor = true; rev.motorSpeed = 1.5f; rev.maxMotorTorque = 200.f;
```

---

## 12. `clone()` / copia de joints y `World`

### 12.1 `clone()` polimórfico

Cada clase implementa `clone()` creando una **copia por valor** de sí misma:

```cpp
[[nodiscard]] std::unique_ptr<Joint> clone() const override {
    return std::make_unique<Distance>(*this);  // idem en las 9 clases
}
```

`joint.hpp` es la única fuente de verdad: si añades un joint nuevo, implementa
`clone()`, `prepare()` y `solve()`.

### 12.2 Copia de `World` clona los joints

El constructor de copia y `operator=` recorren `joints_` y hacen `jointPtr->clone()`
de cada uno. Los cuerpos se clonan también por valor. El test
`testCopyWorldKeepsJoints` (tests.cpp) verifica:

* `World copy = world` conserva `jointCount() == 1`;
* la copia cuelga el cuerpo igual que el original;
* `assigned = world` también clona los joints.

**Consecuencia práctica:** puedes ramificar la simulación (guardar estado,
reproducir desde un checkpoint, rollback) con una simple copia del `World`.
El movimiento (`operator=`) transfiere los `unique_ptr` sin clonar.

**Ojo:** clonar un joint **no** clona los cuerpos que referencia: `a`/`b` son
solo índices (`int`). Al copiar el `World` entero los índices siguen
apuntando a los cuerpos clonados correspondientes, porque el orden de
`bodies_` se preserva.

---

## 13. Solvers sueltos (`constraints.hpp`)

Las funciones libres resuelven **una sola iteración** de un joint entre dos
cuerpos sueltos, sin `World`, sin contactos, sin substeps. Útiles para
física ad-hoc, prototipado, o cuando quieres controlar el bucle tú mismo.
Todas delegan en las clases de `joint.hpp`: misma matemática, un solo origen.

Patrón común:

```cpp
float invDt = dt > 1e-8f ? 1.f/dt : 60.f;
JointT joint{0, 1, /*...*/};
joint.beta = beta;            // si aplica
joint.prepare(bodyA, bodyB, invDt);
joint.solve(bodyA, bodyA, bodyB, bodyB);
```

| Función | Delegante | Notas |
|---|---|---|
| `solveDistance(Body&, Body&, float rest, float dt, float beta=0.2)` | `Distance{0,1,rest}` | Anclas en el centro |
| `solveDistance(Body&, Body&, const Manifold&, float dt)` | `Distance{0,1,rest, aLocal, bLocal}` | `aLocal = bodyA.localPoint(m.point)`, `bLocal = bodyB.localPoint(m.point2)`, `rest = max(0, |Δmundo| - depth)` |
| `solveRevolute(Body&, Body&, Vec2 aA, Vec2 aB, float dt, float beta=0.2)` | `Revolute{0,1,aA,aB}` | Anclas explícitas |
| `solveRevolute(Body&, Body&, const Manifold&, float dt)` | `Revolute{0,1,aLocal,bLocal}` | Anclas = `localPoint` del contacto |
| `solveSpring(Body&, Body&, float rest, float stiff, float damp, float dt)` | `Spring{0,1,rest,stiff,damp}` | Sin anclas (centros) |
| `solveSpring(Body&, Body&, const Manifold&, float dt, float stiff, float damp, float rest)` | `Spring` + anclas `localPoint` del manifold | Overload con manifold |
| `solvePrismatic(Body&, Body&, Vec2 axis, float low, float high, Vec2 anchorA, Vec2 anchorB, float dt, float beta=0.2)` | `Prismatic{0,1,axis,low,high}` + anclas | Anclas asignadas tras construir |
| `solveWeld(Body&, Body&, Vec2 anchorA, Vec2 anchorB, float dt, float beta=0.2)` | `Weld{0,1,anchorA,anchorB}` | Anclas explícitas |
| `solveRope(Body&, Body&, float maxLength, float dt, float beta=0.2)` | `Rope{0,1,maxLength}` | Sin anclas (centros) |
| `solveGear(Body&, Body&, float ratio, float dt, float beta=0.2)` | `Gear{0,1,ratio}` | Solo ángulo |
| `solvePulley(Body&, Body&, Vec2 anchorA, Vec2 anchorB, Vec2 groundA, Vec2 groundB, float ratio, float restTotal, float dt, float beta=0.2)` | `Pulley{0,1,anchorA,anchorB,groundA,groundB,ratio}` | Asigna `restTotal` **antes** de `prepare` (el constructor no lo fija) |

**Diferencias clave con `World`:**

1. Una sola iteración (el `World` repite `velocityIterations` veces).
2. No hay substeps ni integración de fuerzas: tú mueves los cuerpos.
3. No hay `a`/`b`: se pasan `0, 1` como ids dummy (no se usan en la matemática).
4. No hay `maxImpulse`/`gamma` configurables más allá de `beta`; si necesitas
   el motor de Revolute o la amortiguación de Distance, usa la clase directamente.
5. `solveDistance`/`solveRevolute`/`solveSpring` tienen overloads que aceptan un
   `Manifold`, para derivar la restricción de un contacto ya calculado
   (las anclas se convierten a locales con `body.localPoint(...)`; el mundo
   del contacto no se pasa como ancla local).

### 13.1 Ejemplo de solver suelto

```cpp
// Un cuerpo colgado de otro sin World (un solo paso por frame):
phys2d::solveDistance(bodyA, bodyB, /*restLength=*/80.f, /*dt=*/1.f/60.f, /*beta=*/0.2f);

// Con manifold (rest derivado del depth actual):
phys2d::solveDistance(bodyA, bodyB, manifold, dt);

// Resorte sin World:
phys2d::solveSpring(bodyA, bodyB, /*rest=*/50.f, /*stiff=*/100.f, /*damp=*/2.f, dt);

// Cuerda que frena la separación a 120 px:
phys2d::solveRope(bodyA, bodyB, /*maxLength=*/120.f, dt);
```

---

## 14. Tabla comparativa rápida

| Joint | Restricción | Tirar | Empujar | Holgura | Rotación | Cámara | Nº restricciones |
|---|---|---|---|---|---|---|---|
| Distance | distancia = rest | ✓ | ✓ | ✗ | libre | — | 1D |
| Revolute | anclas coincidentes | ✓ | ✓ | ✗ (+motor opcional) | libre | — | 2D (+1D angular) |
| Prismatic | deslizamiento axial + topes | ✓ | ✓ | ✗ | libre según anclas | — | 1D perp + 1D axial condicional |
| Pulley | lenA + ratio·lenB = restTotal | ✓ | ✓ | ✗ | libre | — | 1D |
| Spring | fuerza elástica | ✓ | ✓ (comprimido) | ✗ | libre | — | 1D (soft) |
| Weld | anclas + ángulo | ✓ | ✓ | ✗ | bloqueada | — | 2D + 1D angular |
| Gear | θB + ratio·θA = phase | — | — | ✗ | acoplada | — | 1D angular |
| Rope | distancia ≤ max | ✓ | ✗ | ✓ | libre | — | 1D unilateral |
| Slider | = Prismatic | ✓ | ✓ | ✗ | libre | — | 1D + 1D condicional |

`Rope` es el único unilateral "sin coste" (no actúa hasta que se tensa).
`Gear` es el único puramente angular. `Slider` es en todo binario idéntico
a `Prismatic`.

---

## 15. Ejemplos completos

### 15.1 Péndulo

Versión con `Distance` (la cuerda) y con `Revolute` (la bisagra). Cualquiera
sirve; `Revolute` es más rígido en posición (coincidencia 2D completa).

```cpp
#include "phys2d/phys2d.hpp"
#include <print>

int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    // Ancla estática en (400,100)
    int const anchor = world.addStaticBox(
        phys2d::BoxParams{.pos = {400.f, 100.f}, .half = {30.f, 10.f}});

    // Masa a 120 px, desplazada para que oscile
    int const bob = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {520.f, 100.f}, .radius = 14.f,
                             .linearDamping = 0.001f, .angularDamping = 0.001f});

    // Bisagra: el ancla local del péndulo es (0,-120) bajo el pivote
    (void)world.addRevoluteJoint(anchor, bob, {0.f, 0.f}, {0.f, -120.f});
    // Alternativa con cuerda/de distancia exacta:
    // (void)world.addDistanceJoint(anchor, bob, 120.f);

    for (int i = 0; i < 600; ++i) {
        world.step(1.f / 60.f);
        if (i % 120 == 0) {
            auto const& b = world.body(bob);
            std::println("t={:.2f}s pos={:.1f},{:.1f} vel={:.1f},{:.1f}",
                         i / 60., b.pos.x, b.pos.y, b.vel.x, b.vel.y);
        }
    }
    // La masa cuelga a ~120 px del pivote y oscila con periodo
    // T ≈ 2π·sqrt(L/g) ≈ 2π·sqrt(120/900) ≈ 2.3 s.
    return 0;
}
```

> Para un péndulo con **motor** (reloj, brazo oscilante forzado), añade
> `enableMotor`/`motorSpeed`/`maxMotorTorque` como en §3.

### 15.2 Polea (contrapeso)

Dos masas unidas por una cuerda que pasa sobre dos poleas fijas en el techo.
Si A baja 50 px, B sube 50 px con `ratio = 1`. Con `ratio = 2`, B sube el
doble de rápido (ventaja mecánica).

```cpp
#include "phys2d/phys2d.hpp"
#include <print>

int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    int const a = world.addDynamicBox(
        phys2d::BoxParams{.pos = {300.f, 200.f}, .half = {15.f, 15.f}, .vel = {0.f, 20.f}});
    int const b = world.addDynamicBox(
        phys2d::BoxParams{.pos = {500.f, 300.f}, .half = {15.f, 15.f}});

    // Poleas en (300,0) y (500,0); anclas en los centros; ratio 1
    (void)world.addPulleyJoint(a, b, {0.f, 0.f}, {0.f, 0.f},
                               {300.f, 0.f}, {500.f, 0.f}, /*ratio=*/1.f);

    for (int i = 0; i < 120; ++i) world.step(1.f / 60.f);

    auto const& bodyA = world.body(a);
    auto const& bodyB = world.body(b);
    float const lenA = (bodyA.pos - phys2d::Vec2{300.f, 0.f}).length();
    float const lenB = (bodyB.pos - phys2d::Vec2{500.f, 0.f}).length();
    std::println("lenA+lenB = {:.1f} (esperado ~500)", lenA + lenB);
    std::println("A.y={:.1f} (>200: baja)  B.y={:.1f} (<300: sube)", bodyA.pos.y, bodyB.pos.y);
    return 0;
}
```

Comportamiento sin gravedad (caso del test `testPulleyKeepsTotalLength`,
tests.cpp): A baja y B sube con la rapidez que le da su velocidad inicial,
y la suma `lenA + lenB` se conserva (±8 px de tolerancia).

> **`restTotal`:** si no lo asignas, Pulley congela la suma actual en el
> primer `prepare`. Si quieres una longitud total objetivo distinta de la
> inicial, asigna `restTotal` al joint antes del primer step (accediendo al
> joint por su índice, ver §11.4).

### 15.3 Muelle (amortiguador)

Un cuerpo unido a un ancla por un resorte que lo lleva a `restLength` y
amortigua la oscilación. Ideal para suspensiones, rebotes suaves, trampolines.

```cpp
#include "phys2d/phys2d.hpp"
#include <print>

int main() {
    phys2d::World world;
    world.gravity = {0.f, 0.f};

    int const anchor = world.addStaticBox(
        phys2d::BoxParams{.pos = {100.f, 100.f}, .half = {10.f, 10.f}});
    int const b = world.addDynamicCircle(
        phys2d::CircleParams{.pos = {200.f, 100.f}, .radius = 10.f});

    // rest 50, rigidez 100 N/m, amortiguación 2 N·s/m
    (void)world.addSpringJoint(anchor, b, /*rest=*/50.f,
                               /*stiffness=*/100.f, /*damping=*/2.f);

    for (int i = 0; i < 600; ++i) {
        world.step(1.f / 60.f);
        if (i % 60 == 0) {
            auto const& body = world.body(b);
            std::println("x={:.1f} (objetivo 150)", body.pos.x);
        }
    }
    // Sin amortiguación oscila alrededor de x=150; con damping=2 tiende a él.
    return 0;
}
```

Variantes:

* **Suspensión de coche:** dos `Spring` (delantera/trasera) entre chasis y
  cada rueda, con `stiffness` alta (1000+) y `damping` proporcional a
  `2·sqrt(stiffness·mass)` (amortiguación crítica).
* **Trampolín:** `Spring` vertical entre una plataforma estática y el cuerpo,
  con `stiffness` moderada y `damping = 0` para rebotar.
* **Tenedor suave / imán:** con `stiffness` muy baja actúa como imán elástico.

### 15.4 Cadena

Cadena de N eslabones colgando de un ancla, unidos por `Distance` (o por
`Rope`, si quieres holgura entre eslabones). Es el ejemplo `ex_chain.cpp`.

```cpp
#include "phys2d/phys2d.hpp"
#include <print>
#include <vector>

int main() {
    phys2d::World world;
    world.gravity = {0.f, 900.f};

    int const anchor = world.addStaticBox(
        phys2d::BoxParams{.pos = {400.f, 100.f}, .half = {30.f, 10.f}});

    std::vector<int> ids;
    ids.reserve(8U);
    for (int i = 0; i < 8; ++i) {
        ids.push_back(world.addDynamicCircle(
            phys2d::CircleParams{.pos = {400.f, 130.f + static_cast<float>(i) * 30.f},
                                 .radius = 10.f,
                                 .linearDamping = 0.05f, .angularDamping = 0.05f}));
    }

    float const rest = 30.f;
    world.addJoint<phys2d::Distance>(anchor, ids[0], rest);
    for (std::size_t i = 0U; i + 1U < ids.size(); ++i) {
        world.addJoint<phys2d::Distance>(ids[i], ids[i + 1U], rest);
    }

    for (int i = 0; i < 240; ++i) world.step(1.f / 60.f);

    for (std::size_t i = 0U; i < ids.size(); ++i) {
        auto const& body = world.body(ids[i]);
        std::println("link{} pos={:.1f},{:.1f}", i, body.pos.x, body.pos.y);
    }
    std::println("cola y~{:.1f} (esperado ~{:.1f})",
                 world.body(ids.back()).pos.y, 100.f + 8.f * rest);
    return 0;
}
```

Consejos para cadenas:

* **Más iteraciones:** las cadenas largas son el peor caso del solver
  secuencial (el error se acumula). Sube `world.velocityIterations` (de 8 a
  16–32) o substeps si la cadena se estira.
* **`Rope` vs `Distance`** en cadena: Rope permite que los eslabones se
  acerquen sin resistencia (movimiento de "colgante flojo") y solo frena al
  estirar. Para una cadena real, mezcla: Rope para la articulación
  (movimiento libre hasta `maxLength`) y colisión entre eslabones (no se
  cruzan).
* **Barras en vez de bolas:** usa `addDynamicBox` con `fixedRotation = false`
  y `Revolute` entre extremos para una cadena articulada rígida.

---

## 16. Referencia rápida de tests

Los tests de `tests/tests.cpp` cubren cada joint:

| Test | Joint | Qué verifica |
|---|---|---|
| `testDistanceJointHangs` (88) | Distance | Cuelga a `restLength` |
| `testSpringReachesRestLength` (104) | Spring | Converge a `restLength` |
| `testRopeStopsSeparation` (133) | Rope | Frena la separación en `maxLength`, velocidad acotada |
| `testGearKeepsRatio` (150) | Gear | `ωB + ratio·ωA ≈ 0`, sin explosión |
| `testPulleyKeepsTotalLength` (167) | Pulley | `lenA+lenB ≈ 500`, A baja / B sube |
| `testJointHelpersAcceptBraces` (188) | Revolute | Helper acepta `{0,0}` sin `Vec2{}` explícito |
| `testCopyWorldKeepsJoints` (205) | Distance (copia) | Copiar el World clona los joints |

---

## 17. Fuentes

* `include/phys2d/joint.hpp` — `Joint`, `JointType`, las 9 clases de joint,
  `pointVelocity` / `applyImpulse` / `applyAngularImpulse`.
* `include/phys2d/constraints.hpp` — `solveDistance`, `solveRevolute`,
  `solveSpring`, `solvePrismatic`, `solveWeld`, `solveRope`, `solveGear`,
  `solvePulley`.
* `include/phys2d/world.hpp` — `World::addJoint`, `addJoint<JointT>`,
  `addDistanceJoint`, `addRevoluteJoint`, `addWeldJoint`, `addPrismaticJoint`,
  `addSliderJoint`, `addPulleyJoint`, `addSpringJoint`, `addRopeJoint`,
  `addGearJoint`; copia de `World` (clona `joints_` vía `clone()`).
* `tests/tests.cpp` — tests de cada tipo de joint.
* `examples/ex_chain.cpp` — cadena de 8 eslabones con `Distance`.
