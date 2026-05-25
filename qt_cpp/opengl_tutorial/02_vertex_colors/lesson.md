# Lesson 2: Vertex Colors

## What you will learn

- Multiple vertex attributes (position and color)
- `varying` variables passed from vertex to fragment shader
- Color interpolation across a triangle

## Concepts

- **Attribute** — Per-vertex input (`coord`, `vert_color`).
- **Varying** — Vertex shader output replicated and interpolated per fragment.
- Two buffers must have the **same number of vertices** so attributes align.

## Walkthrough

1. Nine vertices = three triangles (red, green, blue).
2. `vert_color` is set per vertex in the vertex shader and read in the fragment shader as `frag_color`.
3. `Program::Use` enables both attribute arrays by name.

## Build and run

```bash
cmake --build . --target opengl_lesson_02
./opengl_tutorial/02_vertex_colors/opengl_lesson_02
```

## Expected result

Three solid-colored triangles on a dark background.

## Exercises

1. Give each corner of one triangle a different color (RGB corners) and observe smooth blending inside the triangle.
2. Draw a fourth triangle below the others using the same buffers (add 3 vertices + 3 colors).

## Next lesson

[Lesson 3: Uniform transform](../03_uniform_transform/lesson.md)
