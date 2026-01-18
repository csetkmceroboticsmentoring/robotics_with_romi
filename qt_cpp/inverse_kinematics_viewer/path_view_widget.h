#pragma once

#include <memory>

#include <QMutex>
#include <QQueue>
#include <QTimer>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include "./../helper_opengl/buffer.h"
#include "./../helper_opengl/texture.h"
#include "./../helper_opengl/program.h"

class PathViewWidget : public QOpenGLWidget, private QOpenGLFunctions {
  Q_OBJECT
public:
  PathViewWidget(QWidget* parent);

  void clear();
  void add(const float yaw, const std::vector<Eigen::Vector2f>& points);
  void setWaypointEnabled(bool enabled) { waypoint_enabled = enabled; }

signals:
  void newWayPoint(const double, const double);

private:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int width, int height) override;
  void keyPressEvent(QKeyEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;

  void initBackground();
  void initRobotCoords();
  void drawBackground();

  void drawRobot();
  void drawWaypoints();
  void drawTrajectory();

  float scale_factor = 1.0f;
  bool waypoint_enabled = false;

  Eigen::Matrix3f matrix = Eigen::Matrix3f::Identity();

  // Background buffer (rings + cross)
  opengl::ArrayBuffer<Eigen::Vector2f>* bg_buffer = nullptr;

  opengl::Program* program = nullptr;
  opengl::ArrayBuffer<Eigen::Vector2f>* wp_buffer = nullptr;
  opengl::ArrayBuffer<Eigen::Vector2f>* traj_buffer = nullptr;

  // Robot rendering with texture
  opengl::Program* texture_program = nullptr;
  opengl::ArrayBuffer<Eigen::Vector2f>* robot_buffer = nullptr;
  opengl::ArrayBuffer<Eigen::Vector2f>* robot_tex_buffer = nullptr;
  opengl::ElementArrayBuffer<unsigned short>* robot_index_buffer = nullptr;
  opengl::Texture2D* robot_texture = nullptr;
};
