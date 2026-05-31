# OpenGL Tutorial (helper_opengl)

Hands-on lessons for learning 2D and 3D OpenGL through the [`helper_opengl`](../helper_opengl/) library and Qt's `QOpenGLWidget` (lessons 1–7 are 2D; lessons 8–9 are 3D). No prior OpenGL experience required.

## Start here

1. [intro_opengl.md](intro_opengl.md) — what OpenGL is, pipeline, vocabulary (read first)
2. [00_prerequisites.md](00_prerequisites.md) — install, build, Qt lifecycle
3. Lesson 1 and onward — runnable code

## Build all lessons

**Option A — tutorials only** (from this directory):

```bash
cd qt_cpp/opengl_tutorial
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

**Option B — full `qt_cpp` tree** (with other apps):

```bash
cd qt_cpp
mkdir -p build && cd build
cmake ..
cmake --build . -j$(nproc)
```

Requires: Qt5 (Widgets, OpenGL), Eigen3, CMake 3.8+.

Executables: `opengl_lesson_01` … `opengl_lesson_09` in `build/<lesson_dir>/` (paths differ slightly between A and B).

## Run a lesson

After **Option A**:

```bash
./01_hello_triangle/opengl_lesson_01
./08_rotating_triangle_3d/opengl_lesson_08
./09_obj_viewer/opengl_lesson_09
```

After **Option B**:

```bash
./opengl_tutorial/01_hello_triangle/opengl_lesson_01
```

## Lessons

| # | Folder | Topic | Executable |
|---|--------|--------|------------|
| 0 | — | Setup, concepts | — |
| 1 | [01_hello_triangle](01_hello_triangle/) | First triangle, shaders, `Program::Use` | `opengl_lesson_01` |
| 2 | [02_vertex_colors](02_vertex_colors/) | Two attributes (position + color) | `opengl_lesson_02` |
| 3 | [03_uniform_transform](03_uniform_transform/) | `uniform mat3` scale with keyboard | `opengl_lesson_03` |
| 4 | [04_indexed_quad](04_indexed_quad/) | `ElementArrayBuffer`, `glDrawElements` | `opengl_lesson_04` |
| 5 | [05_texture_quad](05_texture_quad/) | `Texture2D`, textured quad | `opengl_lesson_05` |
| 6 | [06_dynamic_line_strip](06_dynamic_line_strip/) | `GL_DYNAMIC_DRAW`, growing polyline | `opengl_lesson_06` |
| 7 | [07_mini_path_viewer](07_mini_path_viewer/) | Grid + trajectory (capstone) | `opengl_lesson_07` |
| 8 | [08_rotating_triangle_3d](08_rotating_triangle_3d/) | 3D triangle, roll/pitch/yaw with Eigen | `opengl_lesson_08` |
| 9 | [09_obj_viewer](09_obj_viewer/) | Load `shuttle.obj`, shaded mesh, mouse rotate | `opengl_lesson_09` |

Each folder contains `lesson.md` (concepts and exercises) and source with comments. Lesson 9 loads a bundled mesh automatically.

## Python track

Eight matching lessons using [`helper_py`](../../python_simulations/helper_py/): [`python_simulations/opengl_tutorial/`](../../python_simulations/opengl_tutorial/). Concepts and exercises are the same; run with `python opengl_tutorial/NN_.../main.py`.

## Reference

- Library API: [`helper_opengl/README.md`](../helper_opengl/README.md)

<p align="center">
  <img src="lesson_07.gif" width="49%" />
  <img src="lesson_09.gif" width="49%" />
</p>
