# Visual demos — Linux x86_64

Ready-to-run builds of the raylib visual examples:

```bash
./visual_vehicle
./visual_stack
./visual_bullet
```

The raylib and GLFW shared libraries are bundled in `.visual-libs/`. These
binaries target Linux x86_64 and still require glibc plus the system X11 and
OpenGL runtime libraries. They are not intended for macOS, Windows, or other
CPU architectures.

Controls:

- Vehicle: arrow keys accelerate/brake; Space pauses; R resets.
- Stack: left-click throws a circle, right-click throws a box; Space pauses;
  R resets.
- CCD comparison: F fires a pair of projectiles, arrow keys change their speed,
  Space pauses, and R resets.

`RAYLIB-COPYRIGHT` and `GLFW-COPYRIGHT` contain the copyright and license
notices for the bundled third-party libraries.
