#include "obj_parser.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

#include <Eigen/Geometry>

namespace ObjParser {

using Eigen::Vector2f;
using Eigen::Vector3f;

namespace {

std::string trim(const std::string& s) {
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

int resolveIndex(const int index, const size_t count) {
  if (index > 0) {
    return index - 1;
  }
  if (index < 0) {
    return static_cast<int>(count) + index;
  }
  return -1;
}

struct FaceCorner {
  int vi = 0;
  int vti = 0;
  int vni = 0;
};

bool parseFaceCorner(const std::string& token, FaceCorner& corner) {
  corner = {};
  const size_t slash1 = token.find('/');
  if (slash1 == std::string::npos) {
    corner.vi = std::stoi(token);
    return true;
  }

  corner.vi = std::stoi(token.substr(0, slash1));
  const size_t slash2 = token.find('/', slash1 + 1);
  if (slash2 == std::string::npos) {
    corner.vti = std::stoi(token.substr(slash1 + 1));
    return true;
  }
  if (slash2 == slash1 + 1) {
    corner.vni = std::stoi(token.substr(slash2 + 1));
    return true;
  }

  corner.vti = std::stoi(token.substr(slash1 + 1, slash2 - slash1 - 1));
  corner.vni = std::stoi(token.substr(slash2 + 1));
  return true;
}

void appendTriangle(const std::shared_ptr<MeshData>& mesh_data,
                    const std::vector<Vector3f>& vertex,
                    const std::vector<Vector3f>& normal,
                    const std::vector<Vector2f>& tex_coord,
                    const FaceCorner& c0,
                    const FaceCorner& c1,
                    const FaceCorner& c2) {
  const int i0 = resolveIndex(c0.vi, vertex.size());
  const int i1 = resolveIndex(c1.vi, vertex.size());
  const int i2 = resolveIndex(c2.vi, vertex.size());
  if (i0 < 0 || i1 < 0 || i2 < 0 ||
      i0 >= static_cast<int>(vertex.size()) ||
      i1 >= static_cast<int>(vertex.size()) ||
      i2 >= static_cast<int>(vertex.size())) {
    return;
  }

  mesh_data->vertex.push_back(vertex[i0]);
  mesh_data->vertex.push_back(vertex[i1]);
  mesh_data->vertex.push_back(vertex[i2]);

  if (c0.vti != 0 && c1.vti != 0 && c2.vti != 0 && !tex_coord.empty()) {
    const int t0 = resolveIndex(c0.vti, tex_coord.size());
    const int t1 = resolveIndex(c1.vti, tex_coord.size());
    const int t2 = resolveIndex(c2.vti, tex_coord.size());
    if (t0 >= 0 && t1 >= 0 && t2 >= 0 &&
        t0 < static_cast<int>(tex_coord.size()) &&
        t1 < static_cast<int>(tex_coord.size()) &&
        t2 < static_cast<int>(tex_coord.size())) {
      mesh_data->tex_coord.push_back(tex_coord[t0]);
      mesh_data->tex_coord.push_back(tex_coord[t1]);
      mesh_data->tex_coord.push_back(tex_coord[t2]);
    }
  }

  if (c0.vni != 0 && c1.vni != 0 && c2.vni != 0 && !normal.empty()) {
    const int n0 = resolveIndex(c0.vni, normal.size());
    const int n1 = resolveIndex(c1.vni, normal.size());
    const int n2 = resolveIndex(c2.vni, normal.size());
    if (n0 >= 0 && n1 >= 0 && n2 >= 0 &&
        n0 < static_cast<int>(normal.size()) &&
        n1 < static_cast<int>(normal.size()) &&
        n2 < static_cast<int>(normal.size())) {
      mesh_data->normal.push_back(normal[n0]);
      mesh_data->normal.push_back(normal[n1]);
      mesh_data->normal.push_back(normal[n2]);
      return;
    }
  }

  const Vector3f u = vertex[i1] - vertex[i0];
  const Vector3f v = vertex[i2] - vertex[i0];
  const Vector3f n = u.cross(v);
  mesh_data->normal.push_back(n);
  mesh_data->normal.push_back(n);
  mesh_data->normal.push_back(n);
}

void parseFaceLine(const std::string& line,
                   const std::shared_ptr<MeshData>& mesh_data,
                   const std::vector<Vector3f>& vertex,
                   const std::vector<Vector3f>& normal,
                   const std::vector<Vector2f>& tex_coord) {
  std::vector<FaceCorner> corners;
  std::istringstream iss(line.substr(2));
  std::string token;
  while (iss >> token) {
    FaceCorner corner;
    try {
      if (!parseFaceCorner(token, corner) || corner.vi == 0) {
        continue;
      }
    } catch (const std::exception&) {
      continue;
    }
    corners.push_back(corner);
  }

  if (corners.size() < 3) {
    return;
  }

  for (size_t i = 1; i + 1 < corners.size(); ++i) {
    appendTriangle(mesh_data, vertex, normal, tex_coord,
                   corners[0], corners[i], corners[i + 1]);
  }
}

std::shared_ptr<MeshData> ensureCurrentMesh(std::shared_ptr<ObjData>& obj_data) {
  if (obj_data->mesh_data_list.empty()) {
    auto mesh_data = std::make_shared<MeshData>();
    mesh_data->group_name = "default";
    obj_data->mesh_data_list.emplace_back(mesh_data);
  }
  return obj_data->mesh_data_list.back();
}

void finalizeMaterial(const std::shared_ptr<Material>& material) {
  if (!material) {
    return;
  }
  if (!material->kd_set && material->ke_set) {
    material->Kd = material->Ke;
  }
  if (!material->kd_set && material->ka_set) {
    material->Kd = material->Ka;
  }
  if (!material->ka_set && material->kd_set) {
    material->Ka = material->Kd;
  }
  if (!material->kd_set && !material->ka_set && material->ke_set) {
    material->Kd = material->Ke;
    material->Ka = material->Ke;
  }
}

bool parseColorLine(const std::string& line,
                    const char* key,
                    Eigen::Vector3f& color,
                    bool& color_set) {
  if (line.compare(0, std::strlen(key), key) != 0) {
    return false;
  }
  float r = 0.0f, g = 0.0f, b = 0.0f;
  if (std::sscanf(line.c_str() + std::strlen(key), " %f %f %f", &r, &g, &b) == 3) {
    color = Vector3f(r, g, b);
    color_set = true;
    return true;
  }
  return false;
}

std::string meshDisplayName(const std::string& group, const std::string& material_name) {
  if (group.empty() || group == "default") {
    return material_name.empty() ? "default" : material_name;
  }
  if (material_name.empty()) {
    return group;
  }
  return group + " / " + material_name;
}

std::shared_ptr<Material> getOrCreateMaterial(const std::shared_ptr<ObjData>& obj_data,
                                              const std::string& material_name) {
  const auto itr = obj_data->material_map.find(material_name);
  if (itr != obj_data->material_map.end()) {
    return itr->second;
  }
  auto material = std::make_shared<Material>();
  finalizeMaterial(material);
  obj_data->material_map.emplace(material_name, material);
  return material;
}

void beginMeshSection(const std::shared_ptr<ObjData>& obj_data,
                      const std::string& group_name,
                      const std::shared_ptr<Material>& material) {
  if (!obj_data->mesh_data_list.empty() &&
      obj_data->mesh_data_list.back()->vertex.empty()) {
    auto& mesh_data = obj_data->mesh_data_list.back();
    mesh_data->group_name = group_name;
    mesh_data->material = material;
    return;
  }
  auto mesh_data = std::make_shared<MeshData>();
  mesh_data->group_name = group_name;
  mesh_data->material = material;
  obj_data->mesh_data_list.emplace_back(mesh_data);
}

}  // namespace

void parseMtlFile(const std::string& file_dir_path,
                  const std::string& file_name,
                  std::shared_ptr<ObjData> obj_data) {
  const std::string file_path = file_dir_path + "/" + trim(file_name);
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cout << "File: " << file_path << " failed\n";
    return;
  }
  std::string line;
  line.reserve(200);
  std::shared_ptr<Material> material;

  while (std::getline(file, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line.compare(0, 6, "newmtl") == 0) {
      if (material) {
        finalizeMaterial(material);
      }
      const std::string material_name = trim(line.substr(6));
      material = std::make_shared<Material>();
      obj_data->material_map[material_name] = material;
      continue;
    }
    if (!material) {
      continue;
    }
    bool ignored = false;
    if (parseColorLine(line, "Ka", material->Ka, material->ka_set) ||
        parseColorLine(line, "Kd", material->Kd, material->kd_set) ||
        parseColorLine(line, "Ks", material->Ks, ignored) ||
        parseColorLine(line, "Ke", material->Ke, material->ke_set)) {
      continue;
    }
    if (line.compare(0, 5, "illum") == 0) {
      std::sscanf(line.c_str() + 5, " %f", &material->illum);
    } else if (line == "d" || line.compare(0, 2, "d ") == 0) {
      std::sscanf(line.c_str() + 1, " %f", &material->d);
    } else if (line == "Tr" || line.compare(0, 3, "Tr ") == 0) {
      float tr = 1.0f;
      std::sscanf(line.c_str() + 2, " %f", &tr);
      material->d = 1.0f - tr;
    }
  }
  if (material) {
    finalizeMaterial(material);
  }
  for (auto& entry : obj_data->material_map) {
    finalizeMaterial(entry.second);
  }
}

std::shared_ptr<ObjData> load(const std::string& file_dir_path,
                              const std::string& file_name) {
  const std::string file_path = file_dir_path + "/" + file_name;
  auto obj_data = std::make_shared<ObjData>();

  std::ifstream file(file_path);
  if (!file.is_open()) {
    return obj_data;
  }

  std::vector<std::string> lines;
  std::string line;
  line.reserve(200);
  while (std::getline(file, line)) {
    lines.push_back(trim(line));
  }

  for (const auto& obj_line : lines) {
    if (obj_line.compare(0, 6, "mtllib") != 0) {
      continue;
    }
    std::istringstream iss(obj_line.substr(6));
    std::string mtl_file;
    while (iss >> mtl_file) {
      parseMtlFile(file_dir_path, mtl_file, obj_data);
    }
  }

  std::string current_group = "default";
  std::shared_ptr<Material> current_material;

  std::vector<Vector3f> vertex;
  std::vector<Vector3f> normal;
  std::vector<Vector2f> tex_coord;

  for (const auto& obj_line : lines) {
    if (obj_line.empty() || obj_line[0] == '#') {
      continue;
    }
    if (obj_line.size() < 2) {
      continue;
    }
    if (obj_line[0] == 'v' && obj_line[1] == ' ') {
      float vx = 0.0f, vy = 0.0f, vz = 0.0f;
      std::sscanf(obj_line.c_str(), "v %f %f %f", &vx, &vy, &vz);
      vertex.emplace_back(vx, vy, vz);
      obj_data->max_v = std::max(
          obj_data->max_v,
          std::max(std::fabs(vx), std::max(std::fabs(vy), std::fabs(vz))));
    } else if (obj_line[0] == 'v' && obj_line[1] == 'n') {
      float nx = 0.0f, ny = 0.0f, nz = 0.0f;
      std::sscanf(obj_line.c_str(), "vn %f %f %f", &nx, &ny, &nz);
      normal.emplace_back(nx, ny, nz);
    } else if (obj_line[0] == 'v' && obj_line[1] == 't') {
      float tx = 0.0f, ty = 0.0f;
      std::sscanf(obj_line.c_str(), "vt %f %f", &tx, &ty);
      tex_coord.emplace_back(tx, ty);
    } else if (obj_line[0] == 'f' && obj_line[1] == ' ') {
      parseFaceLine(obj_line, ensureCurrentMesh(obj_data), vertex, normal, tex_coord);
    } else if (obj_line[0] == 'o' && obj_line[1] == ' ') {
      obj_data->obj_name = trim(obj_line.substr(1));
    } else if (obj_line[0] == 'g') {
      current_group = trim(obj_line.substr(1));
      if (current_group.empty()) {
        current_group = "group";
      }
      beginMeshSection(obj_data, current_group, current_material);
    } else if (obj_line.compare(0, 6, "usemtl") == 0) {
      const std::string material_name = trim(obj_line.substr(6));
      current_material = getOrCreateMaterial(obj_data, material_name);
      beginMeshSection(obj_data, meshDisplayName(current_group, material_name),
                       current_material);
    }
  }

  obj_data->mesh_data_list.remove_if(
      [](const std::shared_ptr<MeshData>& mesh_data) {
        return mesh_data->vertex.empty();
      });

  for (auto& mesh_data : obj_data->mesh_data_list) {
    if (!mesh_data->material) {
      mesh_data->material = getOrCreateMaterial(obj_data, "default");
    }
    finalizeMaterial(mesh_data->material);
    if (!mesh_data->group_name.empty()) {
      obj_data->mesh_data_map[mesh_data->group_name] = mesh_data;
    }
  }
  return obj_data;
}

}  // namespace ObjParser
