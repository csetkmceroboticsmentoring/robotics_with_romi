/**
 * @file main.cpp
 * @brief Lesson 5 — Textured quad (UV coordinates + sampler2D).
 *
 * - `coord`: vertex position in NDC (rotated with `uniform mat3`).
 * - `tex_coord`: UV in [0,1] for sampling the image on the GPU.
 * - Fragment shader uses texture unit 0 (`uniform sampler2D texture` = 0).
 *
 * Texture: `tkmce.jpeg` if found next to the executable or in this lesson folder;
 * otherwise a procedural checkerboard.
 *
 * Controls (click window for focus):
 *   A / D — rotate counter-clockwise / clockwise
 *   R — reset rotation to 0
 */

#include <cmath>
#include <vector>

#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QKeyEvent>
#include <QImage>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QSurfaceFormat>

#include <Eigen/Dense>

#include "buffer.h"
#include "program.h"
#include "texture.h"

using Eigen::Matrix3f;
using Eigen::Vector2f;
using opengl::ArrayBuffer;
using opengl::ElementArrayBuffer;
using opengl::Program;
using opengl::Texture2D;

static const GLchar kVertexShader[] = R"(
  #version 120
  attribute vec2 coord;
  attribute vec2 tex_coord;
  uniform mat3 mat;
  varying vec2 v_tex_coord;
  void main(void) {
    vec3 p = mat * vec3(coord, 1.0);
    gl_Position = vec4(p.xy, 0.0, 1.0);
    v_tex_coord = tex_coord;
  })";

static const GLchar kFragmentShader[] = R"(
  #version 120
  uniform sampler2D texture;
  varying vec2 v_tex_coord;
  void main(void) {
    gl_FragColor = texture2D(texture, v_tex_coord);
  })";

/** 2D rotation about the origin (angle in radians). */
static Matrix3f rotationMatrix2D(float angle) {
  Matrix3f mat = Matrix3f::Identity();
  mat(0, 0) = std::cos(angle);
  mat(0, 1) = -std::sin(angle);
  mat(1, 0) = std::sin(angle);
  mat(1, 1) = std::cos(angle);
  return mat;
}

static const char kTextureFileName[] = "tkmce.jpeg";

/** Builds an RGBA CPU image used when tkmce.jpeg is missing. */
static QImage makeCheckerboard(int size, int cells) {
  QImage image(size, size, QImage::Format_RGBA8888);
  const int cell = size / cells;
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const bool dark = ((x / cell) + (y / cell)) % 2 == 0;
      const QRgb rgb = dark ? qRgb(40, 40, 50) : qRgb(220, 220, 230);
      image.setPixel(x, y, rgb);
    }
  }
  return image;
}

/** Load tkmce.jpeg from common search paths; fall back to checkerboard. */
static QImage loadTkmceImageOrCheckerboard() {
  QStringList dirs;
  dirs << QCoreApplication::applicationDirPath();
  dirs << QDir::currentPath();
  dirs << QDir::currentPath() + QStringLiteral("/../05_texture_quad");
  dirs << QDir::currentPath() + QStringLiteral("/../../05_texture_quad");

  for (const QString& dir : dirs) {
    const QString path = QDir(dir).absoluteFilePath(QLatin1String(kTextureFileName));
    if (!QFile::exists(path)) {
      continue;
    }
    QImage loaded(path);
    if (!loaded.isNull()) {
      qDebug() << "Texture: loaded" << path << "(" << loaded.width() << "x" << loaded.height() << ")";
      return loaded.convertToFormat(QImage::Format_RGBA8888);
    }
    qDebug() << "Texture: could not decode" << path;
  }

  qDebug() << "Texture:" << kTextureFileName << "not found, using checkerboard";
  return makeCheckerboard(128, 8);
}

class TextureQuadWidget : public QOpenGLWidget, private QOpenGLFunctions {
public:
  explicit TextureQuadWidget(QWidget* parent = nullptr)
    : QOpenGLWidget(parent) {
    setFocusPolicy(Qt::StrongFocus);
  }

protected:
  void initializeGL() override {
    initializeOpenGLFunctions();
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    const std::vector<Vector2f> corners = {
      {-0.5f, -0.4f},
      {0.5f, -0.4f},
      {0.5f, 0.4f},
      {-0.5f, 0.4f},
    };
    // UV origin is bottom-left in OpenGL texture space.
    const std::vector<Vector2f> tex_coords = {
      {0.0f, 1.0f},
      {1.0f, 1.0f},
      {1.0f, 0.0f},
      {0.0f, 0.0f},
    };
    const std::vector<unsigned short> indices = {0, 1, 2, 0, 2, 3};

    vertex_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, corners, this);
    tex_buffer_ = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, tex_coords, this);
    index_buffer_ = new ElementArrayBuffer<unsigned short>(GL_STATIC_DRAW, indices, this);

    program_ = new Program({
      {GL_VERTEX_SHADER, kVertexShader},
      {GL_FRAGMENT_SHADER, kFragmentShader},
    }, this);

    const QImage rgba = loadTkmceImageOrCheckerboard();
    texture_ = new Texture2D(GL_RGBA8,
                             GL_RGBA,
                             GL_UNSIGNED_BYTE,
                             rgba.width(),
                             rgba.height(),
                             this,
                             rgba.constBits());
  }

  void resizeGL(int width, int height) override {
    glViewport(0, 0, width, height);
  }

  void paintGL() override {
    glClear(GL_COLOR_BUFFER_BIT);

    const Matrix3f mat = rotationMatrix2D(angle_);

    Program::Use use(*program_, {"coord", "tex_coord"});
    program_->setAttribute("coord", *vertex_buffer_);
    program_->setAttribute("tex_coord", *tex_buffer_);
    program_->setUniform("mat", mat);

    texture_->bind(GL_TEXTURE0);
    program_->setUniform("texture", 0);

    index_buffer_->bind();
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(index_buffer_->size()),
                   GL_UNSIGNED_SHORT,
                   nullptr);
    index_buffer_->unbind();

    texture_->unbind(GL_TEXTURE0);
  }

  void keyPressEvent(QKeyEvent* event) override {
    constexpr float step = 0.08f;
    switch (event->key()) {
      case Qt::Key_A:
        angle_ += step;
        break;
      case Qt::Key_D:
        angle_ -= step;
        break;
      case Qt::Key_R:
        angle_ = 0.0f;
        break;
      default:
        QOpenGLWidget::keyPressEvent(event);
        return;
    }
    update();
  }

private:
  ArrayBuffer<Vector2f>* vertex_buffer_ = nullptr;
  ArrayBuffer<Vector2f>* tex_buffer_ = nullptr;
  ElementArrayBuffer<unsigned short>* index_buffer_ = nullptr;
  Program* program_ = nullptr;
  Texture2D* texture_ = nullptr;
  float angle_ = 0.0f;
};

int main(int argc, char** argv) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  TextureQuadWidget widget;
  widget.setWindowTitle("Lesson 5: Textured Quad (A/D rotate, R reset)");
  widget.resize(640, 480);
  widget.show();
  return app.exec();
}
