#include "widget_gl.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTemporaryDir>

#include <list>

using Eigen::AngleAxisf;
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector2f;
using Eigen::Vector3f;
using opengl::Program;

namespace {

bool copyResourceToFile(const QString& resource_path, const QString& dest_path) {
  if (QFileInfo::exists(dest_path)) {
    QFile::remove(dest_path);
  }
  return QFile::copy(resource_path, dest_path);
}

}  // namespace

static const char vertex_shader[] = R"(
    #version 120
    attribute vec4 vertex;
    attribute vec4 in_normal;
    uniform vec3 a_color;
    uniform mat4 mvp;
    varying vec3 out_coord;
    varying vec3 out_normal;
    varying vec3 out_color;
    void main(void)
    {
        gl_Position = mvp*vertex;
        out_coord = gl_Position.xyz;
        out_color = a_color;
        out_normal = mat3(mvp)*in_normal.xyz;
    })";

static const char fragment_shader[] = R"(
    #version 120
    varying vec3 out_coord;
    varying vec3 out_normal;
    varying vec3 out_color;
    uniform vec3 d_color;
    uniform vec3 light_pos = vec3(0.0, 0.0, -10.0f);
    void main(void)
    {
        vec3 light_dir = normalize(light_pos - out_coord);
        float li = clamp(dot(normalize(out_normal), light_dir), 0.0f, 1.0f);
        vec3 color = out_color + li*d_color;
        gl_FragColor = vec4(color, 1.0f);
    })";

std::shared_ptr<ObjParser::ObjData> loadShuttleData() {
  if (!QFileInfo::exists(QStringLiteral(":/shuttle/shuttle.obj"))) {
    return nullptr;
  }

  static QTemporaryDir temp_dir;
  if (!temp_dir.isValid()) {
    return nullptr;
  }

  const QString dir = temp_dir.path();
  if (!copyResourceToFile(QStringLiteral(":/shuttle/shuttle.obj"),
                          QDir(dir).filePath(QStringLiteral("shuttle.obj"))) ||
      !copyResourceToFile(QStringLiteral(":/shuttle/vp.mtl"),
                          QDir(dir).filePath(QStringLiteral("vp.mtl")))) {
    return nullptr;
  }

  return ObjParser::load(dir.toStdString(), "shuttle.obj");
}
    
WidgetGL::MeshData::MeshData(const ObjParser::MeshData& group, QObject* parent)
  : visible(true),
    group_name(group.group_name),
    vertex_buffer(GL_STATIC_DRAW, group.vertex, parent),
    normal_buffer(GL_STATIC_DRAW, group.normal, parent) {
  if (group.material) {
    material = *group.material;
    if (material.Ka.squaredNorm() < 1e-8f && material.Kd.squaredNorm() > 1e-8f) {
      material.Ka = material.Kd;
    }
  }
};

WidgetGL::WidgetGL(QWidget* parent)
  : QOpenGLWidget(parent) {
  QSurfaceFormat format;
  format.setRenderableType(QSurfaceFormat::OpenGL);
  format.setDepthBufferSize(32);
  setFormat(format);
}

void WidgetGL::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDepthMask(GL_TRUE);

  static const std::list<std::pair<GLenum, const GLchar*>> shaders = {
      { GL_VERTEX_SHADER, vertex_shader },
      { GL_FRAGMENT_SHADER, fragment_shader } };

  program = std::make_unique<Program>(shaders, this);

  if (const auto obj_data = loadShuttleData()) {
    uploadMeshData(obj_data);
  } else {
    qWarning("shuttle.obj not found in Qt resources (:/shuttle/).");
  }
}

void WidgetGL::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
}

void WidgetGL::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!program) {
    return;
  }

  const float safe_scale = (scale_factor > 1e-6f) ? scale_factor : 1.0f;
  const Matrix3f rotation = q.toRotationMatrix() / safe_scale;

  for (const auto& item : faces_map) {
    const auto& mesh_data = item.second;
    if (!mesh_data->visible || mesh_data->vertex_buffer.size() == 0) {
      continue;
    }

    Program::Use use(*program, {"vertex", "in_normal"});
    Matrix4f mat = Matrix4f::Identity();
    mat.block<3, 3>(0, 0) = rotation;
    program->setUniform("mvp", mat);
    program->setUniform("a_color", mesh_data->material.Ka.x(),
                        mesh_data->material.Ka.y(), mesh_data->material.Ka.z());
    program->setUniform("d_color", mesh_data->material.Kd.x(),
                        mesh_data->material.Kd.y(), mesh_data->material.Kd.z());
    program->setAttribute("vertex", mesh_data->vertex_buffer);
    program->setAttribute("in_normal", mesh_data->normal_buffer);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(mesh_data->vertex_buffer.size()));
  }
}

void WidgetGL::uploadMeshData(const std::shared_ptr<ObjParser::ObjData>& obj_data) {
  if (!obj_data) {
    return;
  }
  faces_map.clear();
  scale_factor = 2.0f * std::ceil(obj_data->max_v);
  if (scale_factor < 1e-6f) {
    scale_factor = 1.0f;
  }
  for (const auto& mesh_data : obj_data->mesh_data_list) {
    faces_map.emplace(mesh_data->group_name,
                      std::make_shared<MeshData>(*mesh_data, this));
  }
}

Vector3f WidgetGL::screenCoordToVector(const int x, const int y) {
  const Vector2f half_screen(width() / 2.0f, height() / 2.0f);
  const Vector2f p =
      Vector2f(x - half_screen.x(), half_screen.y() - y).cwiseQuotient(half_screen);
  const float r2_xy = p.squaredNorm();
  const float z = (r2_xy >= 1.0f) ? 0.0f : std::sqrt(1.0f - r2_xy);
  return Vector3f(p.x(), p.y(), z).normalized();
}

void WidgetGL::mousePressEvent(QMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    v1 = screenCoordToVector(event->x(), event->y());
  }
}

void WidgetGL::mouseMoveEvent(QMouseEvent* event) {
  if (event->buttons() & Qt::LeftButton) {
    v2 = screenCoordToVector(event->x(), event->y());
    const float angle = std::acos(v1.dot(v2));
    const Vector3f n = v1.cross(v2).normalized();
    if (std::isnan(angle) || n.norm() == 0.0f) {
      return;
    }
    v1 = v2;
    q = (AngleAxisf(angle, n) * q).normalized();
    repaint();
  }
}

void WidgetGL::mouseReleaseEvent(QMouseEvent *event) {
  if (event->buttons() & Qt::LeftButton) {
    v2 = screenCoordToVector(event->x(), event->y());
    const float angle = std::acos(v1.dot(v2));
    const Vector3f n = v1.cross(v2).normalized();
    if (std::isnan(angle) || n.norm() == 0.0f) {
      return;
    }
    q = (AngleAxisf(angle, n) * q).normalized();
    repaint();
  }
}
