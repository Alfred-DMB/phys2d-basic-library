# Collision — Narrowphase, Solver y Queries (phys2d)

> Documentación extensa en español de `include/phys2d/collision.hpp` y `include/phys2d/shapes.hpp`, con apoyo en `body.hpp`, `world.hpp` y `query.hpp`. Mundo **y-down** (gravedad `+y`, p. ej. `World::gravity{0,900}`).

---

## 1. Manifold: qué devuelve la colisión

`struct Manifold` (alias `using Contact = Manifold;` por compatibilidad con `world.hpp`) es el contacto entre dos cuerpos:

| Campo | Tipo | Significado |
|---|---|---|
| `normal` | `Vec2` (def. `{1,0}`) | **Apunta siempre de A → B**, en ambos órdenes de llamada. Ver §2. |
| `depth` | `float` | Penetración: cuánto se solapan. `0` solo para segmentos (§5.4). |
| `point` | `Vec2` | Primer punto de contacto (mundo). |
| `point2` | `Vec2` | Segundo punto (solo polígono↔polígono, `count==2`; si no, copia de `point`). |
| `count` | `int` (def. 0) | `1` o `2`. `0` por defecto: un `collide()` fallido no deja un `count` inventado. |
| `restitution` | `float` | Combinación de los cuerpos: `eA * eB` (ver §7). |
| `friction` | `float` | Combinación: `sqrt(muA * muB)` (ver §7). |
| `a`, `b` | `int` | IDs de cuerpo; los pone `World` (`-1` en tests manuales). |
| `relVel` | `Vec2` | `B.vel − A.vel` (sin términos angulares). |

`Projection{min,max}` guarda el intervalo de un polígono proyectado sobre un eje (SAT, §5.1).

**Convención de signos del solver:** el solver empuja a B a lo largo de `+normal` (fuera de A) y a A en `-normal`, repartiendo cada punto por su masa efectiva y su torque. Invertir la normal invierte el resultado, por eso la convención A→B es *fundamental* en todo el archivo.

---

## 2. Convención A→B y por qué el dispatcher no invierte

Cabecera de `collision.hpp` (líneas 18–28):

> `Manifold::normal` apunta **siempre** de cuerpo A a cuerpo B. Cada función por pares detecta qué forma tiene cada argumento (orden-agnostic) y escribe `A→B` en **ambos** órdenes. El dispatcher `collide(A,B,out)` **no debe** voltear la normal tras llamar al par.

Ejemplo canónico del comentario (suelo = caja A, bola = círculo B, bola encima):

- `collide(suelo, bola)` → normal `(0,-1)` (suelo→bola, hacia arriba en y-down); el solver empuja a B (bola) en `+normal`, es decir, hacia arriba, fuera del suelo.
- `collide(bola, suelo)` → normal `(0,+1)` (bola→suelo).

Cada función trabaja con una **normal canónica** y al final orienta:

- `circleVsConvex`, `capsuleVsConvex`, `circleAABB`, etc. reciben `bool circleIsA/capsuleIsA/…` y hacen `out.normal = aCircle ? -canonical : canonical;`. Canónico = “dirección que empuja a la forma no-convexa” (por ejemplo círculo fuera del polígono).
- `convexVsConvex` orienta con `if (bestAxis.dot(delta)<0) bestAxis=-bestAxis;` donde `delta = centreB-centreA`, y a partir de ahí todo es A→B.

**Error típico a evitar:** añadir un `out.normal = -out.normal` en `collide()` al llamar a un par en orden invertido. Eso rompería la mitad de los pares.

---

## 3. Formas: `Shape` y el espacio local

`Shape = std::variant<Circle, AABB, ConvexPolygon, Capsule, Segment>` (5 alternativas). **Box es alias de `AABB`** (`using Box = AABB;`): la rotación no se guarda en la forma, está en `Body::angle`. Todo el local se transforma con `local.rotated(angle)+pos` (`Body::pos/angle`).

| Forma | Datos locales | Notas |
|---|---|---|
| `Circle` | `radius` | Centrado en el origen. |
| `AABB`/`Box` | `lower, upper` | Se crea con `fromCenterHalf/fromCenterSize/fromLowerUpper`. `half()`/`size()`/`center()`, `testPoint`, `closestPoint`, etc. |
| `ConvexPolygon` | `std::vector<Vec2> verts` | Vértices antihorarios. `signedArea()`, `centroid()` (área ponderada, promedio si área≈0), `massProps(density)`, `isConvex()`, `edgeNormals()` (hacia afuera, corregidas por signo). |
| `Capsule` | `radius`, `length` (altura total con tapones), `dir` (normalizado, def. `{0,1}`) | `capA()/capB()` = semieje `±dir*(midLength/2)` con `midLength = length−2r` (0 si `length≤2r`). `fromSegment(a,b,r)` construye cápsula desde un segmento. |
| `Segment` | `a`, `b`, `radius` (def. 0) | Con `radius==0` es línea sin área (§5.4). `dir/length/direction/normal`, `closestPoint`, `aabb(angle)`. |

`BodyType`: `Static|Kinematic|Dynamic`. Solo `Dynamic` (con masa > 0) tiene `invMass/invInertia > 0`.

Helpers de `shapes.hpp`:

- `shapeArea(shape)`: área (segmento con radio se trata como cápsula; sin radio, 0).
- `boundingRadius(shape)`: radio de la esfera envolvente alrededor del origen local (usado en broadphase y CCD).
- `massProperties(shape, density, type)`: pone `invMass=invInertia=0` salvo `Dynamic` con masa/inercia > 1e-8. Círculo: `0.5·m·r²`; AABB: `m(w²+h²)/12`; cápsula: rectángulo + círculo; segmento con radio: cápsula + traslación al punto medio.
- `transformPoint/localToWorld`, `worldToLocal`, `transformShape`, `shapeAABB(shape,pos,angle)` (para AABB rotado redondea envolviendo las 4 esquinas), `testPoint`, `pointInside(shape,world,pos,angle)`, `closestPoint(shape, local)` (superficie; si el punto está dentro, devuelve el propio punto).
- `kPi`, `MassProps`, y structs de parámetros **aggregates planos** (C++23: todo diseñado *o* todo posicional, no mezclado): `BodyParams`, `CircleParams{{100,50},20}` o `{.pos=…,.radius=…}`, `BoxParams::fromLowerUpper/fromSize/fromAABB`, `CapsuleParams`, `PolygonParams`, `SegmentParams` (este último `Static` por defecto: típico colisor de suelo).

`Body` (`body.hpp`) guarda `pos, vel, force, angle, angVel, torque, mass/invMass, inertia/invInertia, friction (def. 0.3), restitution (0.2), linear/angularDamping (0.01), allowSleep, isSleeping, fixedRotation, bullet, sleepTimer, type, shape`.

---

## 4. Dispatcher `collide()` y los 15 pares

`bool collide(Body& A, Body& B, Manifold& out)` clasifica con `detail::kindOf(body)` (`holds_alternative` → `Kind::{Circle,Box,Aabb,Polygon,Capsule,Segment,Unknown}`; `Aabb` se trata como `Kind::Box`) y delega. No hace I/O, no reserva para círculos, y devuelve `true` solo si hay contacto; si no, `out` queda intacta (los fallos dejan `count = 0` y no tocan `a`/`b`).

Los **15 pares** (conteo: 5 consigo mismas + C(5,2)=10 cruzados = 15):

| # | Par | Función | Nº de contacto |
|---|---|---|---|
| 1 | Círculo–Círculo | `circleCircle` | 1 |
| 2 | Círculo–Box/AABB | `circleAABB` (+ `circleBox` deprecado) | 1 |
| 3 | Círculo–Polígono | `circlePolygon` → `detail::circleVsConvex` | 1 |
| 4 | Círculo–Cápsula | `circleCapsule` | 1 |
| 5 | Círculo–Segmento | `circleSegment` | 1 (toca a distancia ≤ r) |
| 6 | Box–Box | `boxBox` → `convexVsConvex` | 1–2 |
| 7 | Box/AABB–Polígono | `polygonPolygon` → `convexVsConvex` | 1–2 |
| 8 | Polígono–Polígono | `polygonPolygon` → `convexVsConvex` | 1–2 |
| 9 | Cápsula–Cápsula | `capsuleCapsule` | 1 |
| 10 | Cápsula–Polígono | `capsulePolygon` → `capsuleVsConvex` | 1 |
| 11 | Cápsula–Box/AABB | `capsuleBox` → `capsuleVsConvex` | 1 |
| 12 | Cápsula–Segmento | `capsuleSegment` | 1 |
| 13 | Segmento–Segmento | `segmentSegment` | 1 (solo intersección) |
| 14 | Segmento–Polígono | `segmentPolygon` → `capsuleVsConvex(r=0)` | 1 (mismo camino) |
| 15 | Segmento–Box/AABB | `segmentBox` → `segmentPolygon` | 1 |

Todos los pares son **order-agnostic**: detectan qué argumento es cada forma (`extract*Geom` + `bool aX`) y escriben A→B. Eso significa que `collide(a,b)` y `collide(b,a)` no solo coinciden en `true/false`, sino que devuelven normales opuestas. Solo convexo↔convexo llega a 2 puntos (cara contra cara); el resto es honestamente 1 (§5.2).

---

## 5. SAT + clip y demás tests

### 5.1 SAT + clip: `convexVsConvex` (polígono↔polígono, box↔box)

Núcleo genérico sobre `span<const Vec2>` de vértices mundo. Algoritmo:

1. **Ejes**: normales exteriores de cada cara de **ambos** polígonos (`outwardNormal(v1,v2,centroid)`: perpendicular orientada apuntando fuera del centroide; `normalizeOr` como respaldo).
2. **Proyección**: `project(verts, axis)` → `Projection{min,max}` (dot de todos los vértices). Si algún eje separa (`overlap = min(maxA,maxB)−max(minA,minB) < 0`), no hay colisión.
3. **Eje de mínima penetración**: se guarda el `minOverlap` y su `bestAxis/bestFromA`; al final se orienta a A→B con `delta = centreB−centreA`.
4. **Cara de referencia / incidente**: la cara de referencia es la del polígono ganador cuyo normal más se alinea con `refToInc`; la incidente (`findIncidentFace`) es la cara del otro polígono más anti-paralela a la normal de referencia. `findAxis` es el gemelo público que busca la normal de cara más alineada con `toward`.
5. **Clip**: la cara incidente (segmento de 2 puntos) se recorta contra los dos planos laterales de la cara de referencia con `clipSegment` (Sutherland): `clipSegment(p1,p2,planePoint,planeNormal)` conserva el lado `dot(p−planePoint,normal) ≤ 0`. Hay overload `span` que acepta segmento (2 puntos) o lazo poligonal. Tras el primer clip, un `clipped` vacío significa fuera total. Segundo clip contra `refV2` con tangente.
6. **Selección de contactos**: cada punto recortado con `sep=(p−refV1).dot(refNormal) ≤ 1e-4f` entra; `deepest=−sep` máximo refina `depth`. 0 contactos → fallback centro medio con `count=1` y `depth=minOverlap`. 1 → `count=1`; 2 → `point/point2, count=2`.

Esto produce el contacto **estable cara-cara** (2 puntos → torque correcto en pilas). `boxBox` y `polygonPolygon` solo extraen vértices (`extractPolygonGeom`/`extractBoxOrAabbGeom` vía `boxVerts(centre,half,angle)`) y llaman; `boxVerts` rota `(±half)` a mundo.

### 5.2 Círculo contra convexo: `circleVsConvex` / `circleAABB` / `circleCapsule` / `circleSegment`

- **`circleVsConvex`** (círculo↔polígono, §3): para cada arista calcula `sep=(centre−v1)·n` y el punto más cercano sobre la arista. Si `maxSep ≤ 0` el centro está **dentro**: `depth=radius−maxSep` con la cara de máxima separación. Si no, distancia euclídea a la arista más cercana; `dist>radius` → no hay contacto. Punto de contacto = promedio entre superficie del círculo y punto cercano.
- **`circleAABB`** (círculo↔caja): transforma el centro a local de la caja (`rotated(-angle)`), *clamp* a `±half` → `closestWorld`. Si el centro está dentro (`dist2≈0`), sale por el eje de menor holgura (`dx` vs `dy`) con `depth=radius+dx/dy`. Normal canónica = caja→círculo; se flipa si el círculo es A.
- **`circleCapsule`**: punto más cercano del centro sobre el segmento de la cápsula; si `dist>r+rc` fuera; punto = promedio centro-cercano.
- **`circleSegment`**: igual pero contra segmento; colisiona si `dist ≤ r` (toca exactamente).

Todos: `count=1`, `depth=r−dist` (o `r+dx`), `point/point2` idénticos.

### 5.3 Cápsula/segmento contra convexo: `capsuleVsConvex`

Trabaja “polígono→cápsula” canónico (factor `capsuleIsA` flipa al final). Orden:

1. **Vértice del polígono dentro de la cápsula** (`dist(vert, segmento) < r` para algún vértice): normal del punto cercano a la cápsula, `depth=r−bestDist`.
2. **Punto medio del segmento dentro del polígono** (`maxSep ≤ 0` con latido `insideInfo`): normal de la cara más separante, `depth=r−midSep` (para segmento `r=0` cubre el caso “segmento dentro”).
3. **Fuera**: puntos más cercanos entre el segmento de la cápsula y cada arista (`closestPointsSegments`, §5.4); si `dist>r` fuera, si no `depth=r−dist`.

`capsulePolygon/capsuleBox/segmentPolygon/segmentBox` solo extraen geometría y delegan (el segmento entra como cápsula de radio 0).

### 5.4 Segmentos: profundidad 0 y tolerancias

- **`segmentSegment`**: solo colisiona si se **intersectan** (`segmentsIntersect`: producto cruzado + tolerancia 1e-6, más casos paralelos por extremos) **o** se tocan a `dist ≤ 1e-6`. `depth=0`, `count=1`; normal = diferencia de los puntos más cercanos o, si es 0, perpendicular del segmento A orientada hacia el centro de B.
- **`closestPointsSegments`**: mínimo entre dos segmentos (Dan Sunday). Caso degenerado (denominador ~0, paralelos): si los intervalos se solapan, devuelve el **centro del solape** — no un extremo — porque “un extremo genera torque fantasma en cápsulas/paredes”. Si no se solapa, el extremo más cercano. Segmento degenerado (punto) → punto más cercano del otro.
- **`closestPointOnSegment`** con `t` clamp 0..1 (tolerancia 1e-12).

**Consecuencia física:** un segmento estático no “empuja” como una caja fina: no hay penetración que corregir, solo frena y refleja (vía restricción con `depth=0`). Las paredes deben ser cajas de grosor > 0 o cuidarse la tolerancia.

### 5.5 Extractor de geometría (`detail::*Geom`)

Cada `Body` pasa a `*Geom` en mundo aplicando `pos/angle`: `extractCircleGeom` (centro radio), `extractBoxGeom`/`extractBoxOrAabbGeom` (centro, half, ángulo, 4 verts `boxVerts`), `extractPolygonGeom` (verts rotados + centroide), `extractCapsuleGeom` (`a,b` rotados, centro), `extractSegmentGeom`, y `extractConvexVerts` (polígono o caja → verts). `fillCommon(A,B,out)` rellena `restitution/friction` y `relVel`; `normalizeOr(v,fallback)` evita división por cero.

---

## 6. Solver: impulso con torque (World::stepOnce)

El narrowphase genera `Manifold[]`; el solver en `world.hpp` los convierte en impulsos con **torque** (cada punto aplica impulso y momento):

1. **Broadphase** (fuera de este documento): pares candidatos → `findContacts()` llama `collide(A,B)`; si `pos/angle` no cambiaron desde el step anterior y el par persiste, **reutiliza** el `Manifold` previo actualizando `restitution/friction/relVel` (persistencia, §9).
2. **Expansión por punto**: `SolverContact` por `point` y `point2` (1 ó 2 por Manifold) con:
   - `armA = point−A.pos`, `armB = point−B.pos` (brazo del torque).
   - `massNormal = 1 / (invMassA+invMassB+invInertiaA·(armA×n)²+invInertiaB·(armB×n)²)` y `massTangent` análoga con la tangente (`effectiveMass`).
   - `pointVelocity(body,r) = body.vel + body.angVel × r` (cruz 2D).
3. **Pre-solve (bias)**: `vn0 = rel(v)·n`; umbral de restitución `max(gravity·h,1)`; `e = (vn0 < −thresh) ? restitution : 0` → `restitutionBias = −e·vn0`. Así un cuerpo que apenas apoya no “rebota” por la gravedad.
4. **Warm-start (persistencia de impulsos, §9)**: se aplica el impulso acumulado del step anterior antes de iterar.
5. **Iteraciones de velocidad** (`velocityIterations`, def. 8): por contacto, impulso normal `Δ = −(vn − bias)·massNormal`, **acumulado clamp ≥ 0** (no debe “pegar”); impulso tangente `Δt = −vt·massTangent` clamp a cono de fricción `|Δt| ≤ μ·acumNormal` (Coulomb). Cada impulso se aplica con `applyPointImpulse`: traslación `∓imp·invMass`, rotación `imp·arm×invInertia`.
6. **Corrección de posición** (`positionIterations`, def. 3): `eff = depth − 0.25` (margen para no “vivir” expandiendo), si `eff>0`: `corrMag = eff·0.8 / pointCount` (2 puntos → cada uno corrige su mitad) e impulso `normal · corrMag · massNormal` aplicado directamente a `pos/angle` (sin tocar velocidad). `depth` se reduce en consecuencia.

**Substeps adaptativos** (`World::step`): se mide `maxRatio = max(displacement/boundingRadius)` (lineal y rotacional); si > 0.5 se subdivide (`ceil(ratio/0.5)`, máx 128) para que el cuerpo nunca avance más que medio radio por substep. `h = dt/n`. Esto, junto con CCD, es la primera defensa anti-tunneling.

---

## 7. Fricción y restitución (combinación)

En `detail::fillCommon` (llamado por **cada** par al escribir el `Manifold`, y también al reusar uno persistente):

```cpp
out.restitution = bodyA.restitution * bodyB.restitution;       // e
out.friction    = std::sqrt(bodyA.friction * bodyB.friction); // μ
out.relVel      = bodyB.vel - bodyA.vel;
```

- **Restitución** como **producto**: un suelo con `e=0` anula el rebote sea lo que sea que caiga (físicamente “superficie perfectamente amortiguante”). Dos superficies con `e=0.8` dan `0.64`, no `0.8`.
- **Fricción** como **media geométrica**: asimétrica pero justa (`sqrt(0.1·0.9)=0.3`); evita que un par de superficies “lisas pero rugosas” dé 0.
- `relVel` se deja guardado en el `Manifold` persistente para callbacks y depuración (`World::contacts()` expone el span).

El solver usa `e` solo si `vn0 < −threshold` (§6.3) y `μ` como límite del impulso tangente (§6.5). En CCD con impulso directo en bala se vuelve a multiplicar `restitution` (§8).

---

## 8. CCD swept: time-of-impact antes de integrarse

`World::sweepFastBodies(h)` (privado, invocado al final del solve, **antes** de `integratePositions`) previene el **tunneling** de cuerpos rápidos (`bullet==true` o `|vel|·h > boundingRadius`) — sin ella, un bullet cruzaría la pared en un frame porque el contacto solo se detecta tras penetrar.

Flujo:

1. **Escaneo barato**: si ningún cuerpo dinámico no dormido cumple `disp>radius` o `bullet`, retorna sin tocar buffers.
2. **Snapshot** (`startPos/startVel/startAngVel`): el TOI se calcula contra la posición **inicial** del otro cuerpo usando **velocidad relativa**; el otro cuerpo puede moverse, es una aproximación documentada.
3. **Por bala i, por j≠i**: `relVel = vel_i − vel_j`. Casos:
   - **No-círculo** (AABB/polígono contra AABB/polígono): `shapeCastBodyShape(relVel, |relVel|·h, …)` = sweep **exacto** por Minkowski (§10). El resto (cápsula/segmento) barre el **centro** con `shapeCastBody(…, radius=0, other)`.
   - **Círculo↔círculo**: solución cuadrática `|d0+relVel·τ|² = (rA+rB)²`, primer `τ>0`. Rechaza si ya están solapados en t=0.
   - **Círculo↔AABB sin rotar**: el círculo barre un AABB **expandido por el radio** (slab `raycastAABB`); conservador en esquinas (el TOI puede adelantarse, nunca tunela). AABB rotado se omite en esta rama (pasa por la vía genérica de centros).
4. **Selección del mejor TOI**: `bestT ∈ (1e-6,1]`, `dot(relVel,normal)<0` (acercándose), guarda `bestN/bestP/bestOther`.
5. **Avance y respuesta**:
   - Para círculos: posición += `vel·h·bestT` (y para no-círculos también `angle += angVel·h·bestT` salvo `fixedRotation`); impulso en el punto con masa efectiva + restitución (círculos: `impulse=−(1+e)·relN/(invA+invB)`; no-círculos: `effectiveMass` con brazos).
   - Marca `swept_[i]=1`: `integratePositions` salta ese cuerpo (ya avanzó) y `updateSleep` lo deja.
   - Despertar al otro si estaba dormido.

**Limitaciones conocidas**: el TOI de convexo-rotado contra cápsula/segmento depende de `shapeCastBody` (aproximado, §10); el snapshot asume cuerpos lineales (no curva con rotación completa del otro).

---

## 9. Persistencia: manifolds, warm-start y callbacks

- **Reutilización de manifolds** (`findContacts`): los pares se ordenan; si `prevContacts` contiene el mismo par `(a,b)` y `pos/angle` coinciden (epsilon ~1e-6 en píxeles/radianes), no se llama a `collide`: se copia el `Manifold` y solo se refresca `restitution/friction/relVel`. Así un par quieto no re-ejecuta SAT ni pierde su `point/point2/count` (estable, sin jitter de detección), y `curGeom_` lo alimenta para el siguiente step. Si algo se movió, se recomputa.
- **Warm-start** (`warmEntries_`): al final de cada `stepOnce`, las acumuladas `normal1/2,tangent1/2` por par se guardan y, antes del loop de velocidad, se aplican como impulso inicial en el `SolverContact` correspondiente (búsqueda binaria por `pairKey`). Un contacto que existe hace N frames arranca ya cerca de su solución → menos iteraciones para la misma estabilidad. La clave es `pairKey(a,b)=(lo<<32)^hi`; los `solver_` llegan ordenados porque `pairs_` lo está, así que basta mirar la última entrada para fusionar.
- **Callbacks** `onBeginContact/onEndContact`: se disparan **una vez por step** (no por substep). Si hay suscriptores se construyen `prevKeys/curKeys` (`std::unordered_set<long long>`); `begin` = par nuevo, `end` = par que desapareció (con el `Manifold` previo). Los pares sin ningún cuerpo dinámico (estático-estático) no emiten eventos.

---

## 10. ShapeCast y raycast (query.hpp): el barrido reutilizado por CCD

`query.hpp` es **independiente de `World`** (acepta `std::span<const Body*>` de `World::bodiesSpan()` o `std::span<const std::unique_ptr<Body>>` propio del llamante).

Primitivas:

- `overlapAABB`, `bodyAABB(body)` (`shapeAABB`).
- `closestPointOnSegment/Circle/AABB/Polygon/Capsule`, `pointInCircle/Box/Polygon/Capsule`, `pointInShape`, `closestPointOnShape` (delegan en `shapes.hpp`).
- `raycastCircle` (cuadrática, origen dentro → `fraction=0, normal=-dir`), `raycastAABB` (slab, guarda `n` en el eje que entra), `raycastPolygon` (intersección rayo-arista, `t=min`), `raycastSegment`, `raycastCapsule` (2 tapones circulares solo semi-exteriores + 2 lados desplazados), `raycastBody(shape→mundo)`, `raycastPoint/raycastWorld` (mejor `t` sobre un `BodySpan`, `bodyId` = índice), `pointQuery` (primer cuerpo que contiene el punto).
- **`shapeCastCircle(start,dir,maxDist,radius,targetCenter,targetRadius)`** = `raycastCircle(start,dir,maxDist,targetCenter,targetRadius+radius)`: barrer un círculo equivale a raycast del centro sobre radio suma.
- **`shapeCastBody(start,dir,maxDist,radius,targetBody)`**: dispatch por forma del objetivo. Exactitud por forma:
  - Círculo: `shapeCastCircle` (exacto).
  - AABB sin rotar: AABB expandido por `radius` (conservador en esquinas). AABB rotado: **aproximación** — raycast al polígono de 4 verts **sin engordar** (el sweep del centro sin radio).
  - Polígono: aproximación análoga (sin engordar).
  - Cápsula: `raycastCapsule` con `s.radius+radius` (exacto).
  - Segmento: si `radius>0`, cápsula engordada; si no, aproximación sin engordar.
- **`shapeCastShape(start,dir,maxDist,movingVerts,targetVerts)`** exacto para convexo↔convexo por **Minkowski** (solo traslación): `mink = casco({t−m})` vía `convexHull` (monotone chain) — entonces el sweep del punto `start` contra `mink` detecta el contacto; `raycastPolygon` sobre el casco da `fraction` y `normal`. `shapeCastBodyShape(dir,maxDist,movingBody,targetBody)` es la envoltura AABB/polígono↔AABB/polígono (verts del movible relativos a su centro, objetivo en mundo); si alguna forma no es convexa poligonal devuelve miss.
- `shapeCastPoint` barrier best-`t` sobre `BodySpan`.

`RayHit{hit,point,normal,bodyId,fraction}`: `fraction` es `t/maxDist` (0 = en origen). Para shapeCast, el “hit” es el instante en que las formas se tocan traslación pura (sin rotación del movible durante el barrido).

---

## 11. Ejemplos

### 11.1 Colisión manual entre dos cuerpos

```cpp
#include "phys2d/collision.hpp"
using namespace phys2d;

Body floor, ball;
floor.configure({.pos={400,590}, .angle=0});           // tipo Static
floor.shape = AABB::fromCenterHalf({}, {400,10});       // Box
floor.setStatic();
ball.configure({.pos={400,580}, .angle=0});
ball.shape = Circle{20};

Manifold m{};
if (collide(floor, ball, m)) {
    // m.normal ≈ (0,-1)  (floor -> ball, "hacia arriba" en y-down)
    // m.depth  = cuánto se solapa
    // m.point  = punto de contacto
    // m.restitution = floor.restitution * ball.restitution
    // m.friction    = sqrt(floor.friction * ball.friction)
}
```

> Si inviertes el orden (`collide(ball, floor, m)`), `normal` será `(0,+1)` — mismo absoluto, signo opuesto. No lo inviertas a mano.

### 11.2 Pila de cajas (cara-cara, 2 puntos)

```cpp
World w;
w.gravity = {0,900};
auto floor = w.addStaticBox({.pos={400,590}, .half={400,10}});
for (int i=0;i<5;++i)
    w.addDynamicBox({.pos={400, 500 - i*40}, .half={19,19}, .mass=1});

for (int f=0; f<600; ++f) {
    w.step(1.f/60.f);
    // Los pares Box-Box usan SAT+clip -> count==2:
    // cada Manifold da dos SolverContact -> torque correcto, sin volcarse.
}
```

Sin 2 puntos (una sola esquina apoyada) la pila giraría como si estuviera en un pivote único; el clip es lo que lo estabiliza.

### 11.3 Bullet que no tunela una pared fina

```cpp
World w;
int bullet = w.addDynamicCircle({.pos={100,100}, .vel={2000,0}, .mass=1});
w.body(bullet).bullet = true;            // o depende de |vel|*h > radio
w.body(bullet).shape = Circle{8};
int wall = w.addStaticBox({.pos={400,100}, .half={4,100}});

for (int f=0; f<60; ++f) w.step(1.f/60.f);
// sweepFastBodies detecta el TOI antes de que el círculo cruce la pared
// (radio 8, pared de espesor 8) y lo rebota/_detiene en el contacto.
```

### 11.4 Shape cast manual (approaching query.hpp)

```cpp
#include "phys2d/query.hpp"
using namespace phys2d;

// Barrer un círculo de radio 10 desde {0,0} a la derecha hasta 500:
RayHit h = shapeCastCircle({0,0}, {1,0}, 500.f, 10.f, {300,0}, 20.f);
// h.hit == true, h.fraction == 0.90 (250/500... = distancia donde se tocan), normal = (-1,0)
```

Para convexo↔convexo exacto usar `shapeCastBodyShape(dir, maxDist, movingBody, targetBody)`.

### 11.5 Restitución combinada del par

```cpp
// Suelo amortiguante e=0, bola e=0.9 -> producto = 0 -> no rebota.
floor.restitution = 0.f; ball.restitution = 0.9f;
// Manifold.restitution = 0 * 0.9 = 0
// Dos bolas con e=0.8 -> 0.64
```

---

## 12. Límites y buenas prácticas

- **Segmentos sin área** (`radius==0`) solo detectan intersección/tacto (`depth=0`). Nunca sustituyen a una caja en paredes o suelos donde se espera empuje de penetración.
- **Mundo no rotacional salvo CCD parcial**: el sweep de cápsula/segmento contra polígono usa `shapeCastBody(radius=0)` (§8/§10); para balas con rotación rápida, baja `dt` o sube `substeps`.
- **Persistencia es exacta, no por proximidad**: `Manifold` se reutiliza solo si `pos/angle` no cambiaron bit a bit. Un cuerpo que vibra a nivel de ulp recomputa SAT cada frame (correcto pero más caro).
- **Convexidad obligatoria**: polígonos no convexos son rechazados en `World::validateBody` (`isValid()` exige ≥3 verts, área > 1e-8 y `isConvex()`). Concavos → trocéalo.
- **Tolerancias numéricas**: `1e-4f` para quedar dentro de la cara de referencia, `1e-6f` en intersección de segmentos, `1e-8f` para normalizar. Son proporcionales a escalas “pixels/m” del mundo; si trabajas en metros pequeños, baja las tolerancias de forma consistente.
- **Broadphase vs narrowphase**: `collide()` no debe llamarse sobre pares que broadphase ya descartó por `boundingRadius` — `findContacts` además hace un quick `distance2 > (rA+rB)^2` antes de llamar.

---

## 13. Archivos relacionados

- `phys2d/shapes.hpp` — `Shape`, params, `massProperties`, `boundingRadius`, `testPoint`, `closestPoint`, `shapeAABB`.
- `phys2d/collision.hpp` — `Manifold`, `collide`, 15 pares, `project/centroidOf/findAxis/findIncidentFace/clipSegment`, `detail::*`.
- `phys2d/body.hpp` — `Body(pos,vel,angle)`, `warm` helpers `pointVelocity`, `applyPointImpulse`.
- `phys2d/world.hpp` — `sweepFastBodies`, `stepOnce`, `effectiveMass`, warm-start.
- `phys2d/query.hpp` — `raycast*`, `shapeCast*`, `pointInShape`, `convexHull`.
- `phys2d/joint.hpp` — simétrico al solver de contacto (mismos brazos/torques).
