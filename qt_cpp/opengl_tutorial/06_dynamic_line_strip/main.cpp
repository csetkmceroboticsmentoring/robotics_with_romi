/**
 * @file main.cpp
 * @brief Lesson 6 — Dynamic vertex buffer (GL_DYNAMIC_DRAW).
 *
 * Each frame rebuilds a spiral polyline on the CPU and uploads it with
 * ArrayBuffer::load(). Contrasts with GL_STATIC_DRAW in earlier lessons.
 *
 * QTimer drives animation; paintGL always reflects the latest `points_`.
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

using Eigen::Vector2f;
using opengl::ArrayBuffer;
using opengl::Program;

static const GLchar kVertexShader[] = R"(
  #version 120
  attribute vec2 coord;
  void main(void) {
    gl_Position = vec4(coord, 0.0, 1.0);
  })";

static const GLchar kFragmentShader[] = R"(
  #version 120
  uniform vec3 color;
  void main(void) {
    gl_FragColor = vec4(color, 1.0);
  })";

class DynamicLineWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit DynamicLineWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent),
      t_(0.0f) {}

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Pre-allocate GPU capacity; load() fills actual vertex count each frame.
    line_buffer_ = new ArrayBuffer<Vector2f>(GL_DYNAMIC_DRAW, 4096, this);

    program_ = new Program({
      {GL_VERTEX_SHADER, kVertexShader},
      {GL_FRAGMENT_SHADER, kFragmentShader},
    }, this);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this]() {
      t_ += 0.05f;
      update();
    });
    timer_->start(50);
  }

  void resizeGL(int width, int height) override {
    glViewport(0, 0, width, height);
  }

  void paintGL() override {
    glClear(GL_COLOR_BUFFER_BIT);

    points_.clear();
    points_.push_back(Vector2f(0.0f, 0.0f));
    const int segments = 80;
    for (int i = 1; i <= segments; ++i) {
      const float a = static_cast<float>(i) / segments * 4.0f * 3.14159265f + t_;
      const float r = 0.35f * (static_cast<float>(i) / segments);
      points_.push_back(Vector2f(r * std::cos(a), r * std::sin(a)));
    }
    line_buffer_->load(points_);

    Program::Use use(*program_, {"coord"});
    program_->setAttribute("coord", *line_buffer_);
    program_->setUniform("color", 0.9f, 0.15f, 0.2f);
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(line_buffer_->size()));
  }

private:
  ArrayBuffer<Vector2f>* line_buffer_ = nullptr;
  Program* program_ = nullptr;
  QTimer* timer_ = nullptr;
  std::vector<Vector2f> points_;
  float t_;  ///< phase angle for spiral animation
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  DynamicLineWidget widget;
  widget.setWindowTitle("Lesson 6: Dynamic Line Strip");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
