#pragma once

#include <memory>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include "buffer.h"
#include "program.h"
#include "obj_parser.h"

#include <Eigen/Geometry>

// OpenGL widget: shaded mesh display and trackball rotation.
class WidgetGL : public QOpenGLWidget, private QOpenGLFunctions {
  Q_OBJECT
public:
  // GPU buffers for one OBJ group/material.
  struct MeshData {
    // Build GPU buffers from one parsed OBJ group/material (requires active GL context).
    explicit MeshData(const ObjParser::MeshData& group, QObject* parent);

    bool visible;
    std::string group_name;
    ObjParser::Material material;
    opengl::Buffer<Eigen::Vector3f, GL_ARRAY_BUFFER> vertex_buffer;
    opengl::Buffer<Eigen::Vector3f, GL_ARRAY_BUFFER> normal_buffer;
  };

  explicit WidgetGL(QWidget* parent);

private:
  void uploadMeshData(const std::shared_ptr<ObjParser::ObjData>& obj_data);

  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int width, int height) override;

  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

  // Map screen position to a point on the virtual trackball sphere.
  Eigen::Vector3f screenCoordToVector(const int x, const int y);

  float scale_factor = 1.0f;  // model fit from OBJ bounding extent
  Eigen::Vector3f v1 = Eigen::Vector3f::Zero();
  Eigen::Vector3f v2 = Eigen::Vector3f::Zero();
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity();

  std::unique_ptr<opengl::Program> program;
  std::map<std::string, std::shared_ptr<MeshData>> faces_map;
};
