/**
 * @file main.cpp
 * @brief Lesson 3 — 2D uniform transform (`mat3`) for scaling.
 *
 * The vertex shader multiplies each 2D point (as homogeneous vec3) by `mat`.
 * Here `mat` is diagonal scale on X and Y (Lesson 7 uses the same pattern
 * for a global scene scale).
 *
 * Controls: +/= zoom in, − zoom out (click widget for focus).
 */

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

class UniformTransformWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit UniformTransformWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
  }

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Closed square as LINE_STRIP (5 points, first == last).
    const std::vector<Vector2f> square = {
      {-0.25f, -0.25f},
      {0.25f, -0.25f},
      {0.25f, 0.25f},
      {-0.25f, 0.25f},
      {-0.25f, -0.25f},
    };
    line_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, square, this);

    program_ = new Program({
      {GL_VERTEX_SHADER, kVertexShader},
      {GL_FRAGMENT_SHADER, kFragmentShader},
    }, this);
  }

  void resizeGL(int width, int height) override {
    glViewport(0, 0, width, height);
  }

  void paintGL() override {
    glClear(GL_COLOR_BUFFER_BIT);

    Matrix3f mat = Matrix3f::Identity();
    mat(0, 0) = scale_;
    mat(1, 1) = scale_;

    Program::Use use(*program_, {"coord"});
    program_->setAttribute("coord", *line_buffer_);
    program_->setUniform("mat", mat);
    program_->setUniform("color", 0.2f, 0.4f, 0.85f);
    glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(line_buffer_->size()));
  }

  void keyPressEvent(QKeyEvent* event) override {
    if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Equal) {
      scale_ *= 1.1f;
      if (scale_ > 3.0f) scale_ = 3.0f;
      update();
    } else if (event->key() == Qt::Key_Minus) {
      scale_ /= 1.1f;
      if (scale_ < 0.2f) scale_ = 0.2f;
      update();
    }
    QOpenGLWidget::keyPressEvent(event);
  }

private:
  ArrayBuffer<Vector2f>* line_buffer_ = nullptr;
  Program* program_ = nullptr;
  float scale_ = 1.0f;
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  UniformTransformWidget widget;
  widget.setWindowTitle("Lesson 3: Uniform Transform (+/- to scale)");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
