/**
 * @file main.cpp
 * @brief Lesson 1 — First triangle (2D, normalized device coordinates).
 *
 * Pipeline per frame:
 *   clear → bind Program → setAttribute(VBO) → setUniform(color) → glDrawArrays
 *
 * Vertices are already in NDC (−1…1): no model/view/projection matrices yet.
 * See lesson.md and intro_opengl.md.
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
using opengl::Program;

// Vertex: pass 2D position straight to clip space (z=0, w=1).
static const GLchar kVertexShader[] = R"(
  #version 120
  attribute vec2 coord;
  void main(void) {
    gl_Position = vec4(coord, 0.0, 1.0);
  })";

// Fragment: one RGB uniform for the whole triangle.
static const GLchar kFragmentShader[] = R"(
  #version 120
  uniform vec3 color;
  void main(void) {
    gl_FragColor = vec4(color, 1.0);
  })";

class HelloTriangleWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit HelloTriangleWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent) {}

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    const std::vector<Vector2f> vertices = {
      {0.0f, 0.5f},
      {-0.5f, -0.5f},
      {0.5f, -0.5f},
    };
    vbo_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, vertices, this);

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
    program_->setAttribute("coord", *vbo_);
    program_->setUniform("color", 1.0f, 0.0f, 0.0f);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vbo_->size()));
  }

private:
  ArrayBuffer<Vector2f>* vbo_ = nullptr;
  Program* program_ = nullptr;
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  HelloTriangleWidget widget;
  widget.setWindowTitle("Lesson 1: Hello Triangle");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
