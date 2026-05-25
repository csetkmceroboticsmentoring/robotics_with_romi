/**
 * @file main.cpp
 * @brief Lesson 7 — Capstone: static grid + dynamic path + position marker.
 *
 * Combines lessons 3 (mat3), 6 (dynamic buffer), and line/triangle drawing:
 *   - Grid: GL_STATIC_DRAW, GL_LINES
 *   - Trail: GL_DYNAMIC_DRAW, GL_LINE_STRIP, grows each timer tick
 *   - Marker: static body-frame triangle; pose (rotate + translate) via uniform mat3
 *   - Trail: must load() each frame (vertex count grows); marker geometry is fixed
 *
 * Simulation is a simple 2D integrator (heading + speed), not physics-accurate.
 */

#include <cmath>
#include <vector>

#include <QApplication>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QSurfaceFormat>
#include <QTimer>

#include <Eigen/Dense>

#include "buffer.h"
#include "program.h"

using Eigen::Matrix3f;
using Eigen::Vector2f;
using opengl::ArrayBuffer;
using opengl::Program;

static const GLchar kVertexShader[] = R"(
  #version 120
  attribute vec2 coord;
  uniform mat3 mat;
  void main(void) {
    vec3 p = mat * vec3(coord, 1.0);
    gl_Position = vec4(p.xy, 0.0, 1.0);
  })";

static const GLchar kFragmentShader[] = R"(
  #version 120
  uniform vec3 color;
  void main(void) {
    gl_FragColor = vec4(color, 1.0);
  })";

class MiniPathViewerWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit MiniPathViewerWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent),
      heading_(0.0f),
      pos_(Vector2f::Zero()) {}

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    initGrid();
    initMarker();
    traj_buffer_ = new ArrayBuffer<Vector2f>(GL_DYNAMIC_DRAW, 2000, this);
    traj_points_.push_back(Vector2f::Zero());

    program_ = new Program({
      {GL_VERTEX_SHADER, kVertexShader},
      {GL_FRAGMENT_SHADER, kFragmentShader},
    }, this);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() { stepSimulation(); update(); });
    timer_->start(40);
  }

  void resizeGL(int width, int height) override {
    glViewport(0, 0, width, height);
  }

  void paintGL() override {
    glClear(GL_COLOR_BUFFER_BIT);

    const Matrix3f mat = Matrix3f::Identity() * 0.95f;

    drawLines(*grid_buffer_, 0.75f, 0.75f, 0.75f, mat);
    if (traj_buffer_->size() >= 2) {
      drawLines(*traj_buffer_, 0.1f, 0.35f, 0.9f, mat);
    }
    drawPositionMarker(mat);
  }

private:
  /** 11×11 grid of lines in NDC (−1…1). */
  void initGrid() {
    std::vector<Vector2f> points;
    const int num_lines = 11;
    const float spacing = 2.0f / (num_lines - 1);

    for (int i = 0; i < num_lines; ++i) {
      const float x = -1.0f + i * spacing;
      points.push_back(Vector2f(x, -1.0f));
      points.push_back(Vector2f(x, 1.0f));
    }
    for (int i = 0; i < num_lines; ++i) {
      const float y = -1.0f + i * spacing;
      points.push_back(Vector2f(-1.0f, y));
      points.push_back(Vector2f(1.0f, y));
    }
    grid_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, points, this);
  }

  /** Arrowhead in body frame (tip +X); world pose applied in drawPositionMarker via mat3. */
  void initMarker() {
    const float s = 0.03f;
    const std::vector<Vector2f> body = {
      {s, 0.0f},
      {-0.5f * s, 0.5f * s},
      {-0.5f * s, -0.5f * s},
    };
    marker_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, body, this);
  }

  /** Advance position, append to trail, cap history length, upload to GPU. */
  void stepSimulation() {
    heading_ += 0.04f;
    const float speed = 0.012f;
    pos_.x() += speed * std::cos(heading_);
    pos_.y() += speed * std::sin(heading_);

    traj_points_.push_back(pos_);
    if (traj_points_.size() > 500) {
      traj_points_.erase(traj_points_.begin());
    }
    traj_buffer_->load(traj_points_);
  }

  void drawLines(const ArrayBuffer<Vector2f>& buffer,
                 float r, float g, float b,
                 const Matrix3f& mat) {
    Program::Use use(*program_, {"coord"});
    program_->setAttribute("coord", buffer);
    program_->setUniform("mat", mat);
    program_->setUniform("color", r, g, b);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(buffer.size()));
  }

  /** 2D pose matrix: rotate by heading, translate to pos (then multiply by screen mat). */
  static Matrix3f poseMatrix(const float heading, const Vector2f& pos) {
    Matrix3f pose = Matrix3f::Identity();
    const float c = std::cos(heading);
    const float sn = std::sin(heading);
    pose(0, 0) = c;
    pose(0, 1) = -sn;
    pose(1, 0) = sn;
    pose(1, 1) = c;
    pose(0, 2) = pos.x();
    pose(1, 2) = pos.y();
    return pose;
  }

  /** Transform static marker with mat3; no buffer upload per frame. */
  void drawPositionMarker(const Matrix3f& screen_mat) {
    const Matrix3f marker_mat = screen_mat * poseMatrix(heading_, pos_);

    Program::Use use(*program_, {"coord"});
    program_->setAttribute("coord", *marker_buffer_);
    program_->setUniform("mat", marker_mat);
    program_->setUniform("color", 0.9f, 0.2f, 0.15f);
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }

  ArrayBuffer<Vector2f>* grid_buffer_ = nullptr;
  ArrayBuffer<Vector2f>* traj_buffer_ = nullptr;
  ArrayBuffer<Vector2f>* marker_buffer_ = nullptr;
  Program* program_ = nullptr;
  QTimer* timer_ = nullptr;
  std::vector<Vector2f> traj_points_;
  float heading_;
  Vector2f pos_;
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  MiniPathViewerWidget widget;
  widget.setWindowTitle("Lesson 7: Mini Path Viewer");
  widget.resize(720, 720);
  widget.show();
  return app.exec();
}
