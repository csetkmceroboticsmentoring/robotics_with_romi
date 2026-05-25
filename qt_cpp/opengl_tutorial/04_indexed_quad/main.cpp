/**
 * @file main.cpp
 * @brief Lesson 4 — Indexed drawing with ElementArrayBuffer.
 *
 * A quad uses 4 unique corners but 6 indices (two triangles: 0-1-2 and 0-2-3).
 * glDrawElements looks up vertices by index instead of drawing them in order.
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
using opengl::ArrayBuffer;
using opengl::ElementArrayBuffer;
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

class IndexedQuadWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit IndexedQuadWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent) {}

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    const std::vector<Vector2f> corners = {
      {-0.4f, -0.3f},
      {0.4f, -0.3f},
      {0.4f, 0.3f},
      {-0.4f, 0.3f},
    };
    const std::vector<unsigned short> indices = {0, 1, 2, 0, 2, 3};

    vertex_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, corners, this);
    index_buffer_ = new ElementArrayBuffer<unsigned short>(GL_STATIC_DRAW, indices, this);

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

    Program::Use use(*program_, {"coord"});
    program_->setAttribute("coord", *vertex_buffer_);
    program_->setUniform("color", 0.85f, 0.45f, 0.1f);

    index_buffer_->bind();
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(index_buffer_->size()),
                   GL_UNSIGNED_SHORT,
                   nullptr);  // indices start at offset 0 in bound EBO
    index_buffer_->unbind();
  }

private:
  ArrayBuffer<Vector2f>* vertex_buffer_ = nullptr;
  ElementArrayBuffer<unsigned short>* index_buffer_ = nullptr;
  Program* program_ = nullptr;
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  IndexedQuadWidget widget;
  widget.setWindowTitle("Lesson 4: Indexed Quad");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
