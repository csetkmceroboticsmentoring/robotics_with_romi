# Lesson 8 (Python): 3D Pyramid — Roll, Pitch, Yaw

Concepts and exercises: [C++ lesson.md](../../../qt_cpp/opengl_tutorial/08_rotating_triangle_3d/lesson.md).

This lesson mirrors the C++ track:

- Triangular pyramid (four triangles, 12 vertices)
- **`coord`** and **`vert_color`** vertex attributes (parallel `ArrayBuffer`s, same pattern as lesson 2)
- One **`glDrawArrays(GL_TRIANGLES, 0, 12)`** for the whole mesh
- **`mat4 mvp`** uniform; roll / pitch / yaw from the keyboard
- **`GL_DEPTH_TEST`** and **`GL_CULL_FACE`**

## Run

From `python_simulations/`:

```bash
python opengl_tutorial/08_rotating_triangle_3d/main.py
```

Controls: **Q/E** roll, **W/S** pitch, **A/D** yaw, **R** reset (click window for focus).

## Expected result

A colored pyramid on a black background — pure blue base, pure red / pure green / yellow sides — rotating with the keyboard.
