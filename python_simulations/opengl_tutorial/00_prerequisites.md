# Prerequisites (Python track)

## Packages

From `python_simulations/` (use the same Python you run lessons with, e.g. your conda `base` env):

```bash
pip install -r opengl_tutorial/requirements.txt
```

Or individually:

```bash
pip install PySide6 PyOpenGL numpy
```

The import `from OpenGL.GL import ...` comes from the **PyOpenGL** package (`pip install PyOpenGL`). If you see `ModuleNotFoundError: No module named 'OpenGL'`, that package is missing in the active environment.

## Qt OpenGL lifecycle

Same as the C++ tutorial: `initializeGL` → `resizeGL` → `paintGL` on a `QOpenGLWidget`. See [qt_cpp/opengl_tutorial/00_prerequisites.md](../../qt_cpp/opengl_tutorial/00_prerequisites.md) for the full lifecycle and vocabulary.

## Surface format

Lessons call `configure_gl_format()` in [`_common.py`](_common.py): OpenGL 4.1 **compatibility** profile so GLSL **#version 120** works (same as the C++ lessons).

Lesson 8 enables a 24-bit depth buffer for `GL_DEPTH_TEST`.

## Imports

Each `main.py` adds `python_simulations/` to `sys.path` so you can write:

```python
from helper_py import Program, ArrayBuffer, GL_STATIC_DRAW
from helper_py import GL_VERTEX_SHADER, GL_FRAGMENT_SHADER
```

`Program.Use` is a context manager; `set_attribute` / `set_uniform` match the C++ `helper_opengl` names (`setAttribute` / `setUniform`).

## Quick check

```bash
cd python_simulations
python opengl_tutorial/01_hello_triangle/main.py
```

You should see a red triangle on a black background.

## C++ track (optional)

To compare implementations, build and run the matching lesson under [`qt_cpp/opengl_tutorial/`](../../qt_cpp/opengl_tutorial/).
