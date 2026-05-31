# Lesson 9: OBJ Viewer

## What you will learn

- Loading a real mesh from **`shuttle/shuttle.obj`** at startup
- Parsing Wavefront OBJ/MTL in `obj_parser.cpp`
- Uploading parsed triangles to VBOs and drawing mesh groups
- Simple Phong-style lighting in the fragment shader
- Trackball rotation with the mouse

## Wavefront OBJ

### What it is

**OBJ** (Wavefront `.obj`) is a simple, human-readable **3D mesh** format from the 1990s that is still widely used for exchanging geometry between tools. A mesh is a collection of **vertices** (points in 3D space) connected into **faces** (usually triangles or quads) to form a surface.

An OBJ file is plain text: you can open it in any editor. It stores positions, optional normals and texture coordinates, and how faces reference them. It does **not** embed textures as binary blobs — image files stay separate, and surface color is often described in a companion **`.mtl`** (material) file linked by `mtllib`.

OBJ is easy to parse and debug, which makes it a good fit for tutorials and small loaders. It is not ideal for animated characters or very large scenes (no skeleton animation in the core format; files can get big compared to binary formats like glTF).

### How OBJ files are created

You rarely author OBJ by hand except for tiny tests. In practice, meshes are **exported** from another program:

| Source | Typical workflow |
|--------|------------------|
| **3D modeling** (Blender, Maya, 3ds Max, etc.) | Model the object → **Export → Wavefront (.obj)** → optional `.mtl` and texture images |
| **CAD / mechanical design** | Design a part → export a tessellated OBJ mesh for visualization |
| **Scanning / photogrammetry** | Reconstruct a surface from photos → export OBJ point cloud or mesh |
| **Procedural / code** | Script or library writes `v` and `f` lines (common in research and demos) |
| **Online libraries** | Download ready-made `.obj` models (check license before use) |

The sample in this lesson, `shuttle/shuttle.obj`, came from a model library (see the header comment in the file). When you export from Blender, keep **Apply Modifiers** and **Include Normals / UVs** enabled if your viewer needs them; place the `.mtl` next to the `.obj` if materials should load correctly.

### File layout (lines you will see)

Common lines in an OBJ file:

| Line | Meaning |
|------|---------|
| `v x y z` | Vertex position |
| `vn nx ny nz` | Vertex normal |
| `vt u v` | Texture coordinate |
| `f i j k …` | Face — indices into the lists above (1-based; negative counts from the end) |
| `g name` | Group / sub-mesh |
| `usemtl name` | Switch to a named material from the MTL file |
| `mtllib file.mtl` | Material library to load |

A face line like `f 1/2/3 4/5/6 7/8/9` means three corners, each as **vertex / texture / normal** index. Exporters often emit quads or n-gons; this lesson’s loader triangulates them for OpenGL.

MTL files define materials (`newmtl`, `Kd` diffuse color, `Ka`/`Ks` ambient and specular, etc.). Our viewer reads diffuse color and uses it in a simple Phong-style shader.

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
