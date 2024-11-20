#ifndef BASE_CONTAINER_H
#define BASE_CONTAINER_H

#include "../utils/SimConfig.h"
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <Eigen/Dense>
#include <functional>
#include <indicators/progress_bar.hpp>
#include <numeric>
#include <set>
#include <unordered_map>
#include <vector>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;

class BaseContainer {
public:
  BaseContainer(SimConfig &config, bool GGUI = false);

private:
  void object_initialize();
  Eigen::MatrixXd load_fluid_body(
      const std::unordered_map<std::string, nlohmann::json> &fluid_body,
      double pitch);
  Eigen::MatrixXd load_rigid_body(
      const std::unordered_map<std::string, nlohmann::json> &rigid_body,
      double pitch);
  int compute_cube_particle_num(const Eigen::Vector3d &start,
                                const Eigen::Vector3d &end, double space);
  int compute_box_particle_num(const Eigen::Vector3d &lower_corner,
                               const Eigen::Vector3d &cube_size, double space,
                               double thickness);

  SimConfig &cfg;
  bool GGUI;
  double total_time;
  Eigen::Vector3d domain_start;
  Eigen::Vector3d domain_end;
  Eigen::Vector3d domain_size;
  int dim;
  int material_rigid;
  int material_fluid;
  double dx;
  double particle_diameter;
  double dh;
  double particle_spacing;
  double V0;
  int particle_num;
  int max_num_object;
  double grid_size;
  Eigen::Vector3i grid_num;
  double padding;
  bool add_domain_box;
  Eigen::Vector3d domain_box_start;
  Eigen::Vector3d domain_box_size;
  double domain_box_thickness;
  std::vector<Polyhedron> rigid_bodies;
  std::vector<std::unordered_map<std::string, nlohmann::json>> fluid_bodies;
  std::vector<std::unordered_map<std::string, nlohmann::json>> fluid_blocks;
  std::vector<std::unordered_map<std::string, nlohmann::json>> rigid_blocks;
  int fluid_particle_num;
  int rigid_body_particle_num;
  int particle_max_num;
  std::unordered_map<int, std::unordered_map<std::string, nlohmann::json>>
      object_collection;
  std::set<int> object_id_rigid_body;
  std::set<int> object_id_fluid_body;
  std::vector<int> present_object;
};

#endif // BASE_CONTAINER_H