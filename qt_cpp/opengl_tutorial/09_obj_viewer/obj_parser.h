#pragma once

#include <list>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Core>

// Wavefront OBJ/MTL loader and in-memory mesh representation.
namespace ObjParser {

// MTL material colors (Ka ambient, Kd diffuse, Ke emissive).
struct Material {
  float d = 1.0f;
  float illum = 0.0f;
  Eigen::Vector3f Ka = Eigen::Vector3f::Zero();
  Eigen::Vector3f Kd = Eigen::Vector3f(0.5f, 0.5f, 0.5f);
  Eigen::Vector3f Ks = Eigen::Vector3f::Zero();
  Eigen::Vector3f Ke = Eigen::Vector3f::Zero();
  bool ka_set = false;
  bool kd_set = false;
  bool ke_set = false;
};

// One draw batch: geometry for a group/material pair.
struct MeshData {
  std::string group_name;
  std::shared_ptr<Material> material;
  std::vector<Eigen::Vector3f> vertex;
  std::vector<Eigen::Vector3f> normal;
  std::vector<Eigen::Vector2f> tex_coord;
};

// Full parsed OBJ file.
struct ObjData {
  std::string obj_name;
  float max_v = 0.0f;  // max |coordinate| for viewer scaling
  std::list<std::shared_ptr<MeshData>> mesh_data_list;
  std::map<std::string, std::shared_ptr<MeshData>> mesh_data_map;
  std::map<std::string, std::shared_ptr<Material>> material_map;
};

// file_dir_path: directory containing the .obj and referenced .mtl files.
std::shared_ptr<ObjData> load(const std::string& file_dir_path, const std::string& file_name);

};
