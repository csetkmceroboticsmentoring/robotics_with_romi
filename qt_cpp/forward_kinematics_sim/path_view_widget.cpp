#include "path_view_widget.h"

#include <QDebug>
#include <QKeyEvent>
#include <QImage>

#include <array>
#include <cstdlib>

#ifdef _WIN32
#include <Xinput.h> // Include the XInput header
#pragma comment(lib, "Xinput.lib")
#endif

using Eigen::Vector2f;
using Eigen::Matrix3f;

using opengl::Program;
using opengl::ArrayBuffer;
using opengl::Texture2D;
using opengl::ElementArrayBuffer;

constexpr double kPI = 3.14159265358979323846;
constexpr double kWheelDiameter = 0.07;
constexpr double kDistanceBetweenWheels = 0.14;
constexpr uint32_t kEncoderTicksPerRotation = 1440;

double radToDegree(const double theta) {
  return (180.0 * theta) / kPI;
}

double encoderTicksToDistance(const double enc_ticks) {
  return (kWheelDiameter * kPI / kEncoderTicksPerRotation) * enc_ticks;
}

double distanceToEncoderTicks(const double dist) {
  return std::round((kEncoderTicksPerRotation * dist) / (kWheelDiameter * kPI));
}

double encoderTicksToAngle(const double enc_ticks) {
  return encoderTicksToDistance(enc_ticks) / kDistanceBetweenWheels;
}

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
  uniform float fade;
  uniform sampler2D texture;
  varying vec2 v_tex_coord;
  void main(void)
  {
      gl_FragColor = texture2D(texture, v_tex_coord)*fade;
  })";

PathViewWidget::PathViewWidget(QWidget* parent)
  : QOpenGLWidget(parent),
    left_wheel(0.1613, 12.9032, 0.0),
    right_wheel(0.1613, 12.9032, 0.0),
    curr_state(0.0f, Vector2f::Zero()),
    true_state(0.0f, Vector2f::Zero()) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  setFormat(format);
  traj_points.reserve(40);
  true_traj_points.reserve(40);
  
  // Create control widget as overlay
  control_widget = new ControlWidget(this);
  control_widget->setWindowFlags(Qt::Widget); // Make it a child widget
  control_widget->setAttribute(Qt::WA_TranslucentBackground, false);
  
  // Initially hidden - will be shown if no gamepad
  control_widget->hide();
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
  const float robot_size = 0.17f; // Approximate size to match original
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

  traj_buffer = new ArrayBuffer<Vector2f>(GL_DYNAMIC_DRAW, 100000, this);
  true_traj_buffer = new ArrayBuffer<Vector2f>(GL_DYNAMIC_DRAW, 100000, this);
  
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

  timer = new QTimer(this);
  gamepad = new QGamepad(0, this);

  connect(timer, SIGNAL(timeout()), this, SLOT(timeout()));
  timer->setInterval(25);
  timer->start();
}

void PathViewWidget::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
}

void PathViewWidget::resizeEvent(QResizeEvent* event) {
  QOpenGLWidget::resizeEvent(event);
  positionControlWidget();
}

void PathViewWidget::positionControlWidget() {
  if (!control_widget) return;
  
  // Position control widget at bottom center of the view
  const QSize size = control_widget->sizeHint();
  const int x = (width() - size.width()) / 2;
  const int y = height() - size.height() - 20; // 20px margin from bottom
  
  control_widget->move(x, y);
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

void PathViewWidget::drawRobot(const float r, 
                               const float g, 
                               const float b, 
                               const float fade,
                               const Matrix3f& state) {
  if (!robot_texture) return; // Skip if texture not loaded
  
  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  Program::Use use(*texture_program, { "coord", "tex_coord" });
  texture_program->setAttribute("coord", *robot_buffer);
  texture_program->setAttribute("tex_coord", *robot_tex_buffer);
  const Matrix3f mat = state * scale_factor;
  texture_program->setUniform("mat", mat);
  
  robot_texture->bind(GL_TEXTURE0);
  // Set texture sampler uniform (texture unit 0)
  texture_program->setUniform("texture", 0);
  texture_program->setUniform("fade", fade);
  
  robot_index_buffer->bind();
  glDrawElements(GL_TRIANGLES, robot_index_buffer->size(), GL_UNSIGNED_SHORT, nullptr);
  robot_index_buffer->unbind();
  
  robot_texture->unbind(GL_TEXTURE0);
  
  glDisable(GL_BLEND);
}

void PathViewWidget::drawRobotTrajectory(const float r, 
                                         const float g, 
                                         const float b, 
                                         const ArrayBuffer<Vector2f>& buffer) {
  Program::Use use(*program, {"coord"});
  program->setAttribute("coord", buffer);
  program->setUniform("color", r, g, b);
  const Matrix3f mat = Matrix3f::Identity() * scale_factor;
  program->setUniform("mat", mat);
  glDrawArrays(GL_LINE_STRIP, 0, buffer.size());
}

void PathViewWidget::paintGL() {
  drawBackground();
  drawRobot(0.0f, 0.9f, 0.1f, 1.0f, true_pose);
  drawRobot(0.9f, 0.9f, 0.9f, 0.6f, curr_pose);
  drawRobotTrajectory(0.0f, 0.0f, 1.0f, *true_traj_buffer);
  drawRobotTrajectory(1.0f, 0.0f, 0.0f, *traj_buffer);
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

void PathViewWidget::getGamePadInputs(float& axis_left_y, float& axis_right_x) {
#ifdef _WIN32
  XINPUT_STATE state;
  ZeroMemory(&state, sizeof(XINPUT_STATE)); // Initialize state to zeros

  // Get the state of the controller
  DWORD result = XInputGetState(0, &state);

  if (result == ERROR_SUCCESS) {
    axis_left_y = static_cast<float>(state.Gamepad.sThumbLY) / static_cast<float>(std::numeric_limits<short>::max());
    axis_right_x = -static_cast<float>(state.Gamepad.sThumbRX) / static_cast<float>(std::numeric_limits<short>::max());
    // qDebug() << axis_left_y << ", " << axis_right_x;
  } else {
    qDebug() << "Gamepad not found: " << result;
  }
#else
  axis_left_y = gamepad->axisLeftY();
  axis_right_x = gamepad->axisRightX();
#endif
}

bool PathViewWidget::isGamepadConnected() const {
  return gamepad && gamepad->isConnected();
}

void PathViewWidget::timeout() {
  // Show/hide control widget based on gamepad connection
  const bool has_gamepad = isGamepadConnected();
  if (control_widget) {
    if (has_gamepad && control_widget->isVisible()) {
      control_widget->hide();
    } else if (!has_gamepad && !control_widget->isVisible()) {
      positionControlWidget();
      control_widget->show();
    }
  }
  
  // Get input either from gamepad or control widget
  if (has_gamepad) {
    float axis_left_y = 0.0f;
    float axis_right_x = 0.0f;
    getGamePadInputs(axis_left_y, axis_right_x);

    const int target_fw_vel = static_cast<int>(std::ceil(50.0f * std::max<float>(0, axis_left_y)));
    const int target_twist_vel = static_cast<int>(std::ceil(20.0f * axis_right_x));

    left_wheel.setTarget(target_fw_vel - target_twist_vel);
    right_wheel.setTarget(target_fw_vel + target_twist_vel);
  } else if (control_widget) {
    // Read joystick values directly from control widget
    const double forward = control_widget->leftValue();   // 0.0 to 1.0
    const double turn = control_widget->rightValue();     // -1.0 to +1.0
    
    // Calculate left and right wheel velocities
    constexpr double k1 = 1.0;
    constexpr double k2 = 0.5;
    const double left_vel = k1 * forward - k2 * turn;
    const double right_vel = k1 * forward + k2 * turn;
    
    // Scale to target encoder ticks
    const int target_left_vel = static_cast<int>(std::ceil(50.0 * left_vel));
    const int target_right_vel = static_cast<int>(std::ceil(50.0 * right_vel));
    
    left_wheel.setTarget(target_left_vel);
    right_wheel.setTarget(target_right_vel);
  }

  const int left_vel = left_wheel.update();
  const int right_vel = right_wheel.update();
  const float center_vel = static_cast<float>(left_vel + right_vel) / 2.0;
  curr_state.heading += encoderTicksToAngle(right_vel - left_vel);
  const float dist = encoderTicksToDistance(center_vel);

  const Vector2f new_pos = curr_state.pos + Vector2f(dist * cos(curr_state.heading), dist * sin(curr_state.heading));
  true_state.heading += (static_cast<float>(97 + rand() % 3) / 100.0f) * encoderTicksToAngle(right_vel - left_vel);
  const Vector2f true_new_pos = true_state.pos + Vector2f(dist * cos(true_state.heading), dist * sin(true_state.heading));

  if (curr_state.pos != new_pos || true_state.pos != true_new_pos) {
    curr_state.pos = new_pos;
    traj_points.push_back(new_pos);
    true_state.pos = true_new_pos;
    true_traj_points.push_back(true_new_pos);
  }

  curr_pose << std::cos(curr_state.heading), -std::sin(curr_state.heading), curr_state.pos.x(),
               std::sin(curr_state.heading),  std::cos(curr_state.heading), curr_state.pos.y(),
               0,                             0,                            1.0f;

  true_pose << std::cos(true_state.heading), -std::sin(true_state.heading), true_state.pos.x(),
               std::sin(true_state.heading),  std::cos(true_state.heading), true_state.pos.y(),
               0,                             0,                            1.0f;

  if (traj_points.size() >= 4) {
    makeCurrent();
    traj_buffer->append(traj_points);
    true_traj_buffer->append(true_traj_points);
    doneCurrent();
    traj_points.clear();
    true_traj_points.clear();
    repaint();
  }
}
