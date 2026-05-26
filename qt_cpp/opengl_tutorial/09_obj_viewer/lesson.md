# Lesson 9: OBJ Viewer

Loads **`shuttle/shuttle.obj`** at startup. Demonstrates parsing Wavefront OBJ/MTL, uploading triangles to VBOs, simple Phong-style lighting, and trackball rotation with the mouse.

## Build and run

From `opengl_tutorial/build`:

```bash
cmake --build . --target opengl_lesson_09
./09_obj_viewer/opengl_lesson_09
```

`shuttle.obj` and `vp.mtl` are embedded via [`resources.qrc`](resources.qrc) (`:/shuttle/...`). `WidgetGL::initializeGL()` extracts them to a temp folder for the OBJ loader, then uploads GPU buffers.

## Controls

On-screen hint: **Hold left mouse button inside and drag** to rotate the model (virtual trackball).

## Files

| File | Role |
|------|------|
| `obj_parser.cpp` | OBJ/MTL loader |
| `widget_gl.cpp` | `QOpenGLWidget`, shaders, draw all mesh groups |
| `shuttle/shuttle.obj` | Sample mesh |
