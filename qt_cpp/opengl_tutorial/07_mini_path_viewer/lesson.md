# Lesson 7: Mini Path Viewer (Capstone)

## What you will learn

- Combine static grid geometry and dynamic trajectory data
- Reuse the shader and `mat3` uniform pattern from earlier lessons
- A minimal moving trail with a position marker

## Walkthrough

1. **Grid** — Built once in `initGrid()` (vertical + horizontal lines in NDC).
2. **Trajectory** — Simple integrator updates `pos_` each timer tick; `traj_buffer_->load(traj_points_)`.
3. **Position marker** — One static triangle in body frame; each frame only updates `uniform mat3` (rotate by `heading_`, translate to `pos_`). The trail still uses `load()` because its vertices change every step.
4. **`mat` uniform** — Slight scale (`0.95`) applied to all draws.

## Build and run

```bash
cmake --build . --target opengl_lesson_07
./opengl_tutorial/07_mini_path_viewer/opengl_lesson_07
```

## Expected result

Gray grid, blue path growing in a curve, red triangle at the current position.

## Where to go next

- Combine Lesson 5 textures with this capstone (textured marker or background).
- Add keyboard zoom from Lesson 3.
- Read [helper_opengl/README.md](../../helper_opengl/README.md) for API details.

## Exercises

1. Add a second trajectory buffer in green with a different motion model.
2. Add +/- zoom using `mat` scale (Lesson 3).
3. Port the capstone idea to Python using `helper_py` and `example_triangles.py` as a template.

## Congratulations

You have covered the core OpenGL building blocks used in this course: buffers, shaders, uniforms, indices, textures, and dynamic geometry.

## Next lesson

[Lesson 8: 3D rotating triangle (roll, pitch, yaw)](../08_rotating_triangle_3d/lesson.md)
