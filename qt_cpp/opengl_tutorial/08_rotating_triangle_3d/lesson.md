# Lesson 8: 3D Pyramid — Roll, Pitch, Yaw (Eigen3)

## What you will learn

- 3D vertex positions (`Vector3f`, `ArrayBuffer<Vector3f>`)
- A **`mat4 mvp`** uniform: model × view × projection
- Roll, pitch, yaw: explicit Rx, Ry, Rz (cos/sin) in code; Eigen `AngleAxis` shown commented as the easy alternative
- Per-vertex colors (`vert_color` attribute) with one `glDrawArrays` for the whole pyramid
- Depth testing and back-face culling so the pyramid occludes correctly in 3D

## Roll, pitch, yaw (this lesson)

Angles are in **radians**. Rotation matrices multiply as:

**R = Rz(yaw) × Ry(pitch) × Rx(roll)**

| Angle | Axis | Keys |
|-------|------|------|
| Roll | +X | Q / E |
| Pitch | +Y | W / S |
| Yaw | +Z | A / D |

```cpp
Matrix3f rotationX(float a) {
  Matrix3f r;
  r << 1, 0, 0,
       0, std::cos(a), -std::sin(a),
       0, std::sin(a), std::cos(a);
  return r;
}
// rotationY, rotationZ: same pattern (see main.cpp)

const Matrix3f R = rotationZ(yaw) * rotationY(pitch) * rotationX(roll);
```

**Easy approach** (uncomment in `main.cpp`, add `#include <Eigen/Geometry>`):

```cpp
const AngleAxisf roll_ax(roll, Vector3f::UnitX());
const AngleAxisf pitch_ax(pitch, Vector3f::UnitY());
const AngleAxisf yaw_ax(yaw, Vector3f::UnitZ());
return (yaw_ax * pitch_ax * roll_ax).matrix();
```

Same result; Eigen hides the cos/sin matrix entries. The explicit version is for learning what each matrix contains.

The 3×3 block is placed in a `Matrix4f` **model** matrix. **View** moves the scene back on Z; **projection** is a simple perspective matrix.

## Controls

| Key | Action |
|-----|--------|
| Q / E | Decrease / increase roll |
| W / S | Increase / decrease pitch |
| A / D | Decrease / increase yaw |
| R | Reset angles to zero |

Click the window first for keyboard focus.

## Build and run

```bash
cmake --build . --target opengl_lesson_08
./opengl_tutorial/08_rotating_triangle_3d/opengl_lesson_08
```

## Expected result

A triangular pyramid in 3D with a different color on each face (pure blue base, pure red / pure green sides, yellow fourth side). Use the keyboard to change roll, pitch, and yaw.

## Exercises

1. Change triangle vertices so it is not flat in the XY plane at rest (e.g. add z on one corner).
2. Add a second uniform color and draw the triangle twice with different yaw offsets.
3. Replace manual perspective with an orthographic projection and compare the look.

## Previous lesson

[Lesson 7: Mini path viewer](../07_mini_path_viewer/lesson.md)
