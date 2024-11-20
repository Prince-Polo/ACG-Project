#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

class SimConfig {
public:
  SimConfig(const std::string &scene_file_path);
  Eigen::Vector3d get_vec3_cfg(const std::string &name,
                               bool enforce_exist = false);
  double get_double_cfg(const std::string &name, bool enforce_exist = false);
  bool get_bool_cfg(const std::string &name, bool enforce_exist = false);
  std::vector<std::unordered_map<std::string, nlohmann::json>>
  get_rigid_bodies();
  std::vector<std::unordered_map<std::string, nlohmann::json>>
  get_rigid_blocks();
  std::vector<std::unordered_map<std::string, nlohmann::json>>
  get_fluid_bodies();
  std::vector<std::unordered_map<std::string, nlohmann::json>>
  get_fluid_blocks();

private:
  nlohmann::json config;
};

#endif // SIM_CONFIG_H