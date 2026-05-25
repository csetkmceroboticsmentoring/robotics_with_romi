# Lesson 3: Uniform Transform

## What you will learn

- **Uniform** — One value for the entire draw call (not per vertex)
- `uniform mat3 mat` to scale 2D geometry (common pattern for zoom and pan)
- Keyboard input triggering `update()` to redraw

## Concepts

- **Uniform** vs **attribute**: uniforms are constant across all vertices in a draw; attributes vary per vertex.
- `mat * vec3(coord, 1.0)` is an affine 2D transform in homogeneous coordinates.

## Controls

- **+** / **=** — zoom in
- **-** — zoom out

Click the window first so it has keyboard focus.

## Build and run

```bash
cmake --build . --target opengl_lesson_03
./opengl_tutorial/03_uniform_transform/opengl_lesson_03
```

## Expected result

A blue square outline that grows and shrinks with +/-.

## Exercises

1. Add translation: offset the square with keys (modify `mat(0,2)` and `mat(1,2)`).
2. Change line color based on `scale_` in `paintGL`.

## Next lesson

[Lesson 4: Indexed quad](../04_indexed_quad/lesson.md)
