/**
 * @file main.cpp
 * @brief Lesson 2 — Per-vertex colors with two vertex attributes.
 *
 * Two parallel buffers (position + color) must have the same vertex count.
 * The vertex shader forwards color via a `varying`; the GPU interpolates it
 * across each triangle before the fragment shader runs.
 */

#include <vector>

#include <QApplication>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QSurfaceFormat>

#include <Eigen/Dense>

#include "buffer.h"
#include "program.h"

using Eigen::Vector2f;
using Eigen::Vector3f;
using opengl::ArrayBuffer;
using opengl::Program;

static const GLchar kVertexShader[] = R"(
  #version 120
  attribute vec2 coord;
  attribute vec3 vert_color;
  varying vec3 frag_color;
  void main(void) {
    gl_Position = vec4(coord, 0.0, 1.0);
    frag_color = vert_color;
  })";

static const GLchar kFragmentShader[] = R"(
  #version 120
  varying vec3 frag_color;
  void main(void) {
    gl_FragColor = vec4(frag_color, 1.0);
  })";

class VertexColorsWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit VertexColorsWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent) {}

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // 9 vertices = 3 triangles (red, green, blue)
    const std::vector<Vector2f> positions = {
      {0.0f, 0.45f}, {-0.45f, -0.35f}, {0.45f, -0.35f},
      {-0.55f, 0.1f}, {-0.85f, -0.45f}, {-0.25f, -0.45f},
      {0.55f, 0.1f}, {0.25f, -0.45f}, {0.85f, -0.45f},
    };
    const std::vector<Vector3f> colors = {
      {1.0f, 0.2f, 0.2f}, {1.0f, 0.2f, 0.2f}, {1.0f, 0.2f, 0.2f},
      {0.2f, 1.0f, 0.3f}, {0.2f, 1.0f, 0.3f}, {0.2f, 1.0f, 0.3f},
      {0.3f, 0.5f, 1.0f}, {0.3f, 0.5f, 1.0f}, {0.3f, 0.5f, 1.0f},
    };

    pos_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, positions, this);
    color_buffer_ = new ArrayBuffer<Vector3f>(GL_STATIC_DRAW, colors, this);

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

    Program::Use use(*program_, {"coord", "vert_color"});
    program_->setAttribute("coord", *pos_buffer_);
    program_->setAttribute("vert_color", *color_buffer_);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(pos_buffer_->size()));
  }

private:
  ArrayBuffer<Vector2f>* pos_buffer_ = nullptr;
  ArrayBuffer<Vector3f>* color_buffer_ = nullptr;
  Program* program_ = nullptr;
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  VertexColorsWidget widget;
  widget.setWindowTitle("Lesson 2: Vertex Colors");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
