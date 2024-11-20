#include "SimConfig.h"
#include <fstream>
#include <iostream>

SimConfig::SimConfig(const std::string &scene_file_path) {
  std::ifstream file(scene_file_path);
  if (file.is_open()) {
    file >> config;
  } else {
    std::cerr << "Unable to open file: " << scene_file_path << std::endl;
  }
  std::cout << config.dump(4) << std::endl;
}

Eigen::Vector3d SimConfig::get_vec3_cfg(const std::string &name,
                                        bool enforce_exist) {
  if (enforce_exist) {
    assert(config["Configuration"].contains(name));
  }
  if (!config["Configuration"].contains(name)) {
    if (enforce_exist) {
      assert(config["Configuration"].contains(name));
    } else {
      return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
  }
  auto arr = config["Configuration"][name];
  return Eigen::Vector3d(arr[0], arr[1], arr[2]);
}

double SimConfig::get_double_cfg(const std::string &name, bool enforce_exist) {
  if (enforce_exist) {
    assert(config["Configuration"].contains(name));
  }
  if (!config["Configuration"].contains(name)) {
    if (enforce_exist) {
      assert(config["Configuration"].contains(name));
    } else {
      return 0.0;
    }
  }
  return config["Configuration"][name];
}

bool SimConfig::get_bool_cfg(const std::string &name, bool enforce_exist) {
  if (enforce_exist) {
    assert(config["Configuration"].contains(name));
  }
  if (!config["Configuration"].contains(name)) {
    if (enforce_exist) {
      assert(config["Configuration"].contains(name));
    } else {
      return false;
    }
  }
  return config["Configuration"][name];
}

std::vector<std::unordered_map<std::string, nlohmann::json>>
SimConfig::get_rigid_bodies() {
  if (config.contains("RigidBodies")) {
    return config["RigidBodies"]
        .get<std::vector<std::unordered_map<std::string, nlohmann::json>>>();
  } else {
    return {};
  }
}

std::vector<std::unordered_map<std::string, nlohmann::json>>
SimConfig::get_rigid_blocks() {
  if (config.contains("RigidBlocks")) {
    return config["RigidBlocks"]
        .get<std::vector<std::unordered_map<std::string, nlohmann::json>>>();
  } else {
    return {};
  }
}

std::vector<std::unordered_map<std::string, nlohmann::json>>
SimConfig::get_fluid_bodies() {
  if (config.contains("FluidBodies")) {
    return config["FluidBodies"]
        .get<std::vector<std::unordered_map<std::string, nlohmann::json>>>();
  } else {
    return {};
  }
}

std::vector<std::unordered_map<std::string, nlohmann::json>>
SimConfig::get_fluid_blocks() {
  if (config.contains("FluidBlocks")) {
    return config["FluidBlocks"]
        .get<std::vector<std::unordered_map<std::string, nlohmann::json>>>();
  } else {
    return {};
  }
}