# Lesson 0: Prerequisites

Read [intro_opengl.md](intro_opengl.md) first for OpenGL concepts and vocabulary.

## Software

- **Qt 5** with modules: Core, Widgets, OpenGL
- **Eigen3** 3.3+
- **CMake** 3.8+
- **C++14** compiler (GCC, Clang, or MSVC)

On Ubuntu/Debian:

```bash
sudo apt install qtbase5-dev libqt5opengl5-dev libeigen3-dev cmake build-essential
```

## Concepts before Lesson 1

### Qt OpenGL widget lifecycle

`QOpenGLWidget` is a Qt widget that owns an OpenGL context. You override:

| Method | When it runs | Typical work |
|--------|----------------|--------------|
| `initializeGL()` | Once, context ready | Create buffers, shaders, textures |
| `resizeGL(w, h)` | Window resized | `glViewport(0, 0, w, h)` |
| `paintGL()` | Each frame | Clear screen, bind program, draw |

### Normalized device coordinates (NDC)

**Lessons 1–7** use 2D vertex positions from **-1 to +1** (no separate projection matrix):

- (-1, -1) = bottom-left of the window
- (1, 1) = top-right

Lesson 3 adds a simple **`mat3` scale** (still 2D). **Lesson 8** is 3D: positions are `vec3`, and model × view × projection produce NDC.

### Course layout (2D then 3D)

| Lessons | Focus |
|---------|--------|
| 1–7 | 2D: buffers, shaders, textures, dynamic lines, grid + path |
| 8 | 3D: depth buffer, perspective, roll / pitch / yaw |

### Pipeline (one sentence each)

1. **Vertex shader** — moves each vertex; output is clip-space position.
2. **Rasterizer** — turns triangles into fragments (pixels).
3. **Fragment shader** — sets the color of each fragment.
4. **Framebuffer** — what you see in the window.

### What helper_opengl provides

| Class | Role |
|-------|------|
| `ArrayBuffer<T>` | Vertex data on the GPU (VBO) |
| `ElementArrayBuffer<T>` | Index list for indexed drawing |
| `Program` | Compile/link shaders; set uniforms and attributes |
| `Program::Use` | RAII: activate program and attribute arrays in a scope |
| `Texture2D` | Image on the GPU |

You still call OpenGL draw functions (`glDrawArrays`, `glDrawElements`, `glClear`).

## Build check

```bash
cd qt_cpp/opengl_tutorial
mkdir -p build && cd build
cmake ..
cmake --build . --target opengl_lesson_01
./01_hello_triangle/opengl_lesson_01
```

(Or build from `qt_cpp/build` after `cmake ..` in `qt_cpp` — see [README.md](README.md).)

You should see a red triangle on a black background.

Next: [01_hello_triangle/lesson.md](01_hello_triangle/lesson.md)
