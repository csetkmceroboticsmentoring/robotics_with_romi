# Lesson 5: Textured Quad

## What you will learn

- `Texture2D` creation from CPU image data
- Texture coordinates (`tex_coord`) from 0 to 1
- `sampler2D` and binding to `GL_TEXTURE0`
- `uniform mat3` to rotate the quad with the keyboard (vertices spin; UVs stay fixed)

## Concepts

- **Texture** — 2D image on the GPU sampled by `(u, v)` coordinates.
- **Texture unit** — Slot (`GL_TEXTURE0`, …); the uniform `texture` is set to `0` meaning unit 0.

Place **`tkmce.jpeg`** in this folder (`05_texture_quad/`). The app loads it if found; otherwise it uses a procedural checkerboard. CMake copies the file beside the executable on build when it exists.

## Controls

| Key | Action |
|-----|--------|
| A | Rotate counter-clockwise |
| D | Rotate clockwise |
| R | Reset rotation |

Click the window first for keyboard focus.

## Build and run

```bash
cmake --build . --target opengl_lesson_05
./opengl_tutorial/05_texture_quad/opengl_lesson_05
```

## Expected result

`tkmce.jpeg` on the quad (or a checkerboard if the file is missing). **A** / **D** rotate, **R** resets.

## Exercises

1. Replace `tkmce.jpeg` with another image (keep the same filename or change `kTextureFileName` in `main.cpp`).
2. Animate: call `texture_->load(...)` each frame with a shifted checkerboard (optional challenge).

## Next lesson

[Lesson 6: Dynamic line strip](../06_dynamic_line_strip/lesson.md)
