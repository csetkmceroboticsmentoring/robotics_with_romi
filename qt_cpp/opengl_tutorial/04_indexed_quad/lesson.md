# Lesson 4: Indexed Quad

## What you will learn

- `ElementArrayBuffer` for index data
- `glDrawElements` instead of `glDrawArrays`
- Why indices save memory when meshes reuse vertices

## Concepts

A rectangle needs only **four** corner vertices. Two triangles need **six** index values:

```
Triangle 1: 0, 1, 2
Triangle 2: 0, 2, 3
```

The GPU looks up `corners[index]` for each index.

## Build and run

```bash
cmake --build . --target opengl_lesson_04
./opengl_tutorial/04_indexed_quad/opengl_lesson_04
```

## Expected result

An orange filled rectangle.

## Exercises

1. Change index order to flip the quad winding; notice if back-face culling matters (culling is off by default here).
2. Add a second smaller quad with another vertex/index buffer pair.

## Next lesson

[Lesson 5: Textured quad](../05_texture_quad/lesson.md)
