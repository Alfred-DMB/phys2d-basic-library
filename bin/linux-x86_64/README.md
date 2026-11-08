# Demos visuales — Linux x86_64

Binarios para Linux de los ejemplos visuales hechos con raylib:

```bash
chmod +x visual_vehicle visual_stack visual_bullet
./visual_vehicle
./visual_stack
./visual_bullet
```

Ejecuta estos comandos desde esta carpeta después de descargar los archivos.

Las bibliotecas compartidas de raylib y GLFW están incluidas en `.visual-libs/`.
Estos binarios son para Linux x86_64 y requieren glibc, además de las
bibliotecas de sistema X11 y OpenGL. No están pensados para macOS, Windows ni
otras arquitecturas.

Controles:

- Vehículo: las flechas aceleran o frenan; Espacio pausa; R reinicia.
- Pila: clic izquierdo lanza un círculo y clic derecho una caja; Espacio pausa;
  R reinicia.
- Comparación de CCD: F dispara dos proyectiles; las flechas ajustan su
  velocidad; Espacio pausa y R reinicia.

`RAYLIB-COPYRIGHT` y `GLFW-COPYRIGHT` contienen los avisos de copyright y las
licencias de las bibliotecas de terceros incluidas.
