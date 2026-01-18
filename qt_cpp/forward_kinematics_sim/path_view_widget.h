#pragma once

#include <memory>

#include <QMutex>
#include <QQueue>
#include <QTimer>
#include <QGamepad>
#include <QResizeEvent>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include "./../helper_opengl/buffer.h"
#include "./../helper_opengl/texture.h"
#include "./../helper_opengl/program.h"
#include "./../virtual_controller/control_widget.h"

#include "wheel_with_encoder.h"

struct State {
  State(const float, const Eigen::Vector2f& pos) : heading(heading), pos(pos) {};
  float heading;
  Eigen::Vector2f pos;
};

class PathViewWidget : public QOpenGLWidget, private QOpenGLFunctions {
  Q_OBJECT
public:
  PathViewWidget(QWidget* parent);

  // Check if gamepad is connected
  bool isGamepadConnected() const;

private slots:
  void timeout();

private:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int width, int height) override;
  void resizeEvent(QResizeEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  
  void positionControlWidget();

  void initBackground();
  void initRobotCoords();

  void drawBackground();

  void drawRobot(const float r, 
                 const float g, 
                 const float b, 
                 const float fade,
                 const Eigen::Matrix3f& state);

  void drawRobotTrajectory(const float r, 
                           const float g, 
                           const float b, 
                           const opengl::ArrayBuffer<Eigen::Vector2f>& buffer);

  void getGamePadInputs(float& axis_left_y, float& axis_right_x);

  float scale_factor = 1.0f;

  WheelWithEncoder left_wheel;
  WheelWithEncoder right_wheel;

  QTimer* timer = nullptr;
  QGamepad* gamepad = nullptr;
  ControlWidget* control_widget = nullptr;

  // Current state as per odometry.
  State curr_state;
  // True state.
  State true_state;

  std::vector<Eigen::Vector2f> traj_points;
  std::vector<Eigen::Vector2f> true_traj_points;

  Eigen::Matrix3f curr_pose = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f true_pose = Eigen::Matrix3f::Identity();

  // Background buffer (rings + cross)
  opengl::ArrayBuffer<Eigen::Vector2f>* bg_buffer = nullptr;

  opengl::Program* program = nullptr;

  opengl::ArrayBuffer<Eigen::Vector2f>* robot_buffer = nullptr;
  opengl::ArrayBuffer<Eigen::Vector2f>* robot_tex_buffer = nullptr;
  opengl::ElementArrayBuffer<unsigned short>* robot_index_buffer = nullptr;
  opengl::Texture2D* robot_texture = nullptr;
  opengl::Program* texture_program = nullptr;

  opengl::ArrayBuffer<Eigen::Vector2f>* traj_buffer = nullptr;
  opengl::ArrayBuffer<Eigen::Vector2f>* true_traj_buffer = nullptr;
};
