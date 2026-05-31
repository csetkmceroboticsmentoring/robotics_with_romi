/**
 * @file main.cpp
 * @brief Lesson 8 — 3D triangular pyramid with roll, pitch, and yaw (Eigen3).
 *
 * Demonstrates the model–view–projection (MVP) pipeline:
 *   gl_Position = projection * view * model * vec4(coord, 1)
 *
 * - **Model**: 3×3 rotation as Rz * Ry * Rx (explicit cos/sin matrices below).
 *   Eigen AngleAxis is shown commented as the shorter equivalent (#include Eigen/Geometry).
 * - **View**: translates the world along −Z so geometry sits in front of the camera.
 * - **Projection**: perspective matrix (FOV, aspect, near/far) maps eye space to clip space.
 *
 * Coordinates: angles are radians. Pyramid is four triangles in one draw call;
 * per-vertex colors (same color repeated for each triangle's three corners).
 *
 * Controls (widget must have focus):
 *   Q/E — roll −/+    W/S — pitch +/−    A/D — yaw −/+    R — reset angles
 *
 * @see lesson.md and intro_opengl.md (3D rotation section).
 */

#include <cmath>
#include <vector>

#include <QApplication>
#include <QKeyEvent>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QSurfaceFormat>

#include <Eigen/Dense>

#include "buffer.h"
#include "program.h"

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using opengl::ArrayBuffer;
using opengl::Program;

// -----------------------------------------------------------------------------
// Shaders (GLSL 1.20)
// -----------------------------------------------------------------------------

/** Passes 3D position through combined MVP; forwards per-vertex color. */
static const GLchar kVertexShader[] = R"(
  #version 120
  attribute vec3 coord;
  attribute vec3 vert_color;
  uniform mat4 mvp;
  varying vec3 frag_color;
  void main(void) {
    gl_Position = mvp * vec4(coord, 1.0);
    frag_color = vert_color;
  })";

/** Interpolated color from the vertex shader. */
static const GLchar kFragmentShader[] = R"(
  #version 120
  varying vec3 frag_color;
  void main(void) {
    gl_FragColor = vec4(frag_color, 1.0);
  })";

// -----------------------------------------------------------------------------
// Math helpers (Eigen3, column-major — matches OpenGL uniform layout)
// -----------------------------------------------------------------------------

/**
 * @brief OpenGL-style perspective projection matrix.
 * @param fov_y_deg Vertical field of view in degrees.
 * @param aspect    Width / height of the drawable (from resizeGL).
 * @param z_near    Near clip plane (eye-space Z); must be > 0.
 * @param z_far     Far clip plane; must be > z_near.
 * @return 4×4 matrix P such that clip_coords = P * eye_coords.
 *
 * After this, the GPU performs perspective divide (x,y,z divided by w).
 * m(3,2) = −1 stores eye-space Z into w so farther objects project smaller.
 */
static Matrix4f perspective(float fov_y_deg, float aspect, float z_near, float z_far) {
  const float f = 1.0f / std::tan(fov_y_deg * 0.5f * static_cast<float>(M_PI) / 180.0f);
  Matrix4f m = Matrix4f::Zero();
  m(0, 0) = f / aspect;  // scale X for non-square windows
  m(1, 1) = f;           // scale Y from vertical FOV
  m(2, 2) = (z_far + z_near) / (z_near - z_far);
  m(2, 3) = (2.0f * z_far * z_near) / (z_near - z_far);
  m(3, 2) = -1.0f;
  return m;
}

/** @brief Rotation about +X (roll). */
static Matrix3f rotationX(float angle) {
  Matrix3f r;
  r << 1.0f, 0.0f, 0.0f,
       0.0f, std::cos(angle), -std::sin(angle),
       0.0f, std::sin(angle), std::cos(angle);
  return r;
}

/** @brief Rotation about +Y (pitch). */
static Matrix3f rotationY(float angle) {
  Matrix3f r;
  r << std::cos(angle), 0.0f, std::sin(angle),
       0.0f, 1.0f, 0.0f,
       -std::sin(angle), 0.0f, std::cos(angle);
  return r;
}

/** @brief Rotation about +Z (yaw). */
static Matrix3f rotationZ(float angle) {
  Matrix3f r;
  r << std::cos(angle), -std::sin(angle), 0.0f,
       std::sin(angle), std::cos(angle), 0.0f,
       0.0f, 0.0f, 1.0f;
  return r;
}

/**
 * @brief Rotation matrix from roll–pitch–yaw (yaw-pitch-roll order).
 * @param roll  Radians about +X.
 * @param pitch Radians about +Y.
 * @param yaw   Radians about +Z.
 * @return R = Rz(yaw) * Ry(pitch) * Rx(roll).
 */
static Matrix3f rotationFromRollPitchYaw(float roll, float pitch, float yaw) {
  // --- Easy approach (Eigen): one line per axis, multiply, then .matrix() ---
  // Requires: #include <Eigen/Geometry> and using Eigen::AngleAxisf;
  // Eigen builds the same 3×3 matrices internally from cos/sin. Use this in
  // application code once you are comfortable with the axis order (Z * Y * X).
  //
  // const AngleAxisf roll_ax(roll, Vector3f::UnitX());
  // const AngleAxisf pitch_ax(pitch, Vector3f::UnitY());
  // const AngleAxisf yaw_ax(yaw, Vector3f::UnitZ());
  // return (yaw_ax * pitch_ax * roll_ax).matrix();

  // --- Explicit approach (this lesson): see each Rx, Ry, Rz entry ---
  const Matrix3f rx = rotationX(roll);
  const Matrix3f ry = rotationY(pitch);
  const Matrix3f rz = rotationZ(yaw);
  return rz * ry * rx;
}

/** Embeds 3×3 rotation in the top-left of a 4×4 model matrix (no translation). */
static Matrix4f modelMatrix(float roll, float pitch, float yaw) {
  Matrix4f model = Matrix4f::Identity();
  model.block<3, 3>(0, 0) = rotationFromRollPitchYaw(roll, pitch, yaw);
  return model;
}

// -----------------------------------------------------------------------------
// QOpenGLWidget — resource lifetime tied to GL context
// -----------------------------------------------------------------------------

class RotatingTriangle3DWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit RotatingTriangle3DWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
  }

protected:
  /** Create GPU objects once the OpenGL context exists. */
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);  // nearer fragments win when triangles overlap
    glEnable(GL_CULL_FACE);   // hide back-facing triangles as the pyramid spins

    // Triangular pyramid: four triangles (12 vertices), CCW when viewed from outside.
    const Vector3f apex{0.0f, 0.0f, 0.55f};
    const Vector3f b0{-0.55f, -0.35f, -0.35f};
    const Vector3f b1{0.55f, -0.35f, -0.35f};
    const Vector3f b2{0.0f, 0.45f, -0.35f};

    const std::vector<Vector3f> vertices = {
      b0, b1, b2,       // base
      apex, b1, b0,     // side
      apex, b2, b1,     // side
      apex, b0, b2,     // side
    };
    vbo_ = new ArrayBuffer<Vector3f>(GL_STATIC_DRAW, vertices, this);

    const Vector3f blue{0.0f, 0.0f, 1.0f};
    const Vector3f red{1.0f, 0.0f, 0.0f};
    const Vector3f green{0.0f, 1.0f, 0.0f};
    const Vector3f yellow{1.0f, 0.85f, 0.25f};
    const std::vector<Vector3f> colors = {
      blue, blue, blue,
      red, red, red,
      green, green, green,
      yellow, yellow, yellow,
    };
    color_buffer_ = new ArrayBuffer<Vector3f>(GL_STATIC_DRAW, colors, this);

    program_ = new Program({
      {GL_VERTEX_SHADER, kVertexShader},
      {GL_FRAGMENT_SHADER, kFragmentShader},
    }, this);
  }

  /** Match viewport and projection aspect to window size. */
  void resizeGL(int width, int height) override {
    glViewport(0, 0, width, height);
    aspect_ = (height > 0) ? static_cast<float>(width) / static_cast<float>(height) : 1.0f;
  }

  /** One frame: clear, build MVP, draw the pyramid in one call. */
  void paintGL() override {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Projection: camera lens (FOV 45°, clip planes 0.1 … 10).
    const Matrix4f projection = perspective(45.0f, aspect_, 0.1f, 10.0f);

    // View: push world back so pyramid at origin is visible (camera looks down −Z).
    Matrix4f view = Matrix4f::Identity();
    view(2, 3) = -2.8f;

    // Model: roll, pitch, yaw from keyboard.
    const Matrix4f model = modelMatrix(roll_, pitch_, yaw_);

    // Apply transforms right-to-left on column vectors: clip = P * V * M * v.
    const Matrix4f mvp = projection * view * model;

    Program::Use use(*program_, {"coord", "vert_color"});
    program_->setAttribute("coord", *vbo_);
    program_->setAttribute("vert_color", *color_buffer_);
    program_->setUniform("mvp", mvp);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vbo_->size()));
  }

  void keyPressEvent(QKeyEvent* event) override {
    constexpr float step = 0.08f;  // radians per key press
    switch (event->key()) {
      case Qt::Key_Q: roll_ -= step; break;
      case Qt::Key_E: roll_ += step; break;
      case Qt::Key_W: pitch_ += step; break;
      case Qt::Key_S: pitch_ -= step; break;
      case Qt::Key_A: yaw_ -= step; break;
      case Qt::Key_D: yaw_ += step; break;
      case Qt::Key_R:
        roll_ = pitch_ = yaw_ = 0.0f;
        break;
      default:
        QOpenGLWidget::keyPressEvent(event);
        return;
    }
    update();
  }

private:
  ArrayBuffer<Vector3f>* vbo_ = nullptr;
  ArrayBuffer<Vector3f>* color_buffer_ = nullptr;
  Program* program_ = nullptr;

  float aspect_ = 1.0f;   ///< width/height for projection
  float roll_ = 0.0f;     ///< radians, rotation about X
  float pitch_ = 0.0f;    ///< radians, rotation about Y
  float yaw_ = 0.0f;      ///< radians, rotation about Z
};

// -----------------------------------------------------------------------------
// Application entry
// -----------------------------------------------------------------------------

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  format.setDepthBufferSize(24);  // required for GL_DEPTH_TEST
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  RotatingTriangle3DWidget widget;
  widget.setWindowTitle(
    "Lesson 8: 3D Pyramid (Q/E roll, W/S pitch, A/D yaw, R reset)");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
