#include "path_view_widget.h"

#include <QDebug>
#include <QKeyEvent>
#include <QImage>

#include <array>
#include <cstdlib>

using Eigen::Vector2f;
using Eigen::Matrix3f;

using opengl::Program;
using opengl::ArrayBuffer;
using opengl::Texture2D;
using opengl::ElementArrayBuffer;

static const GLchar vertex_shader[] = R"(
  #version 120
  attribute vec2 coord;
  uniform mat3 mat;
  void main(void)
  {
      vec3 new_pos = mat*vec3(coord.xy, 1.0f);
      gl_Position = vec4(new_pos.xy, 0.0f, 1.0f);
  })";

static const GLchar fragment_shader[] = R"(
  #version 120
  uniform vec3 color;
  void main(void)
  {
      gl_FragColor = vec4(color, 1.0f);
  })";

static const GLchar texture_vertex_shader[] = R"(
  #version 120
  attribute vec2 coord;
  attribute vec2 tex_coord;
  uniform mat3 mat;
  varying vec2 v_tex_coord;
  void main(void)
  {
      vec3 new_pos = mat*vec3(coord.xy, 1.0f);
      gl_Position = vec4(new_pos.xy, 0.0f, 1.0f);
      v_tex_coord = tex_coord;
  })";

static const GLchar texture_fragment_shader[] = R"(
  #version 120
  uniform sampler2D texture;
  varying vec2 v_tex_coord;
  void main(void)
  {
      gl_FragColor = texture2D(texture, v_tex_coord);
  })";

PathViewWidget::PathViewWidget(QWidget* parent)
  : QOpenGLWidget(parent) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  setFormat(format);
}

void PathViewWidget::initBackground() {
  // Create a grid with vertical and horizontal lines
  // Grid spans from -1 to +1 in both directions
  // 11 lines in each direction (spacing of 0.2)
  std::vector<Vector2f> points;
  
  const int num_lines = 11;
  const float spacing = 2.0f / (num_lines - 1);
  
  // Vertical lines
  for (int i = 0; i < num_lines; ++i) {
    const float x = -1.0f + i * spacing;
    points.push_back(Vector2f(x, -1.0f));
    points.push_back(Vector2f(x, 1.0f));
  }
  
  // Horizontal lines
  for (int i = 0; i < num_lines; ++i) {
    const float y = -1.0f + i * spacing;
    points.push_back(Vector2f(-1.0f, y));
    points.push_back(Vector2f(1.0f, y));
  }
  
  bg_buffer = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, points, this);
}

void PathViewWidget::initRobotCoords() {
  // Create a quad for the robot image (centered at origin, size similar to original robot)
  const float robot_size = 0.15f; // Approximate size to match original
  const std::vector<Vector2f> robot_points = {
    {-robot_size/2.0f, -robot_size/2.0f}, // Bottom-left
    {robot_size/2.0f, -robot_size/2.0f},  // Bottom-right
    {robot_size/2.0f, robot_size/2.0f},   // Top-right
    {-robot_size/2.0f, robot_size/2.0f}   // Top-left
  };
  const std::vector<Vector2f> tex_coords = {
    {0.0f, 1.0f}, // Bottom-left
    {1.0f, 1.0f}, // Bottom-right
    {1.0f, 0.0f}, // Top-right
    {0.0f, 0.0f}  // Top-left
  };
  const std::vector<unsigned short> indices = {
    0, 1, 2,  // First triangle
    0, 2, 3   // Second triangle
  };
  robot_buffer = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, robot_points, this);
  robot_tex_buffer = new ArrayBuffer<Vector2f>(GL_STATIC_DRAW, tex_coords, this);
  robot_index_buffer = new ElementArrayBuffer<unsigned short>(GL_STATIC_DRAW, indices, this);
}

void PathViewWidget::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

  initBackground();
  initRobotCoords();

  wp_buffer = new ArrayBuffer<Vector2f>(GL_DYNAMIC_DRAW, 100000, this);
  traj_buffer = new ArrayBuffer<Vector2f>(GL_DYNAMIC_DRAW, 100000, this);
  
  static const std::list<std::pair<GLenum, const GLchar*>> shaders = {
    { GL_VERTEX_SHADER, vertex_shader },
    { GL_FRAGMENT_SHADER, fragment_shader} };
  program = new Program(shaders, this);

  // Create texture shader program
  static const std::list<std::pair<GLenum, const GLchar*>> texture_shaders = {
    { GL_VERTEX_SHADER, texture_vertex_shader },
    { GL_FRAGMENT_SHADER, texture_fragment_shader} };
  texture_program = new Program(texture_shaders, this);

  // Load robot texture from Qt resources
  QImage image(":/romi_small.png");
  if (image.isNull()) {
    qDebug() << "Failed to load romi_small.png from Qt resources";
    std::exit(1);
  }
  // Convert to RGBA format
  QImage converted = image.convertToFormat(QImage::Format_RGBA8888);
  const uchar* data = converted.constBits();
  robot_texture = new Texture2D(GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE,
                                converted.width(), converted.height(),
                                this, data);
}

void PathViewWidget::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
}

void PathViewWidget::drawBackground() {
  glClear(GL_COLOR_BUFFER_BIT);
  
  // Draw grid
  Program::Use use(*program, {"coord"});
  program->setAttribute("coord", *bg_buffer);
  program->setUniform("color", 0.3f, 0.3f, 0.3f);
  const Matrix3f mat = Matrix3f::Identity() * scale_factor;
  program->setUniform("mat", mat);
  glDrawArrays(GL_LINES, 0, bg_buffer->size());
}

void PathViewWidget::drawRobot() {
  if (!robot_texture) return; // Skip if texture not loaded
  
  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  Program::Use use(*texture_program, { "coord", "tex_coord" });
  texture_program->setAttribute("coord", *robot_buffer);
  texture_program->setAttribute("tex_coord", *robot_tex_buffer);
  const Matrix3f mat = matrix * scale_factor;
  texture_program->setUniform("mat", mat);
  
  robot_texture->bind(GL_TEXTURE0);
  // Set texture sampler uniform (texture unit 0)
  texture_program->setUniform("texture", 0);
  
  robot_index_buffer->bind();
  glDrawElements(GL_TRIANGLES, robot_index_buffer->size(), GL_UNSIGNED_SHORT, nullptr);
  robot_index_buffer->unbind();
  
  robot_texture->unbind(GL_TEXTURE0);
  
  glDisable(GL_BLEND);
}

void PathViewWidget::drawWaypoints() {
  Program::Use use(*program, { "coord" });
  program->setAttribute("coord", *wp_buffer);
  program->setUniform("color", 0.0f, 0.0f, 1.0f);
  const Matrix3f mat = Matrix3f::Identity() * scale_factor;
  program->setUniform("mat", mat);
  glDrawArrays(GL_LINE_STRIP, 0, wp_buffer->size());
}

void PathViewWidget::drawTrajectory() {
  Program::Use use(*program, { "coord" });
  program->setAttribute("coord", *traj_buffer);
  program->setUniform("color", 1.0f, 0.0f, 0.0f);
  const Matrix3f mat = Matrix3f::Identity() * scale_factor;
  program->setUniform("mat", mat);
  glDrawArrays(GL_LINE_STRIP, 0, traj_buffer->size());
}

void PathViewWidget::paintGL() {
  drawBackground();
  drawRobot();
  drawWaypoints();
  drawTrajectory();
}

void PathViewWidget::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_Up:
      scale_factor += 0.25;
      break;
    case Qt::Key_Down:
      scale_factor -= 0.25;
      break;
  }
  repaint();
}

void PathViewWidget::add(const float yaw, const std::vector<Vector2f>& points) {
  makeCurrent();
  traj_buffer->append(points);
  doneCurrent();
  matrix(0, 0) = cos(yaw);
  matrix(0, 1) = -sin(yaw);
  matrix(1, 0) = sin(yaw);
  matrix(1, 1) = cos(yaw);
  matrix(0, 2) = points.back().x();
  matrix(1, 2) = points.back().y();
  matrix(2, 2) = 1.0f;
  repaint();
}

void PathViewWidget::mousePressEvent(QMouseEvent* event) {
  if (waypoint_enabled && event->button() == Qt::LeftButton) {
    Eigen::Vector2d p;
    p.x() = static_cast<double>(event->x())/static_cast<double>(width()/2.0) - 1.0;
    p.y() = 1.0 - static_cast<double>(event->y())/static_cast<double>(height()/2.0);
    p /= scale_factor;
    emit newWayPoint(p.x(), p.y());

    std::vector<Eigen::Vector2f> points;
    if (wp_buffer->size() == 0) {
      points.emplace_back(matrix(0, 2), matrix(1, 2));
    }
    points.emplace_back(p.cast<float>());
    makeCurrent();
    wp_buffer->append(points);
    doneCurrent();
    repaint();
  }
}

void PathViewWidget::clear() {
  makeCurrent();
  wp_buffer->clear();
  traj_buffer->clear();
  doneCurrent();
  repaint();
}
