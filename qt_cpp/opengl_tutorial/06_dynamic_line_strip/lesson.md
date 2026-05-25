# Lesson 6: Dynamic Line Strip

## What you will learn

- `GL_DYNAMIC_DRAW` buffers that change every frame
- `ArrayBuffer::load()` to upload new vertex data
- `QTimer` to drive animation

## Concepts

Plot and chart applications often keep a growing list of points and upload them to the GPU each frame with `ArrayBuffer::load()`—the same pattern as Lesson 6’s spiral.

## Build and run

```bash
cmake --build . --target opengl_lesson_06
./opengl_tutorial/06_dynamic_line_strip/opengl_lesson_06
```

## Expected result

A red spiral line that grows and rotates over time.

## Exercises

1. Cap the number of points (ring buffer) instead of recomputing all 80 each frame.
2. Draw the path twice: estimated (red) and “true” (green) with two buffers.

## Next lesson

[Lesson 7: Mini path viewer](../07_mini_path_viewer/lesson.md)
