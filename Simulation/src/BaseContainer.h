#ifndef BASE_CONTAINER_H
#define BASE_CONTAINER_H

#include "SimConfig.h"
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

  void insert_object();
  double compute_rigid_body_mass(int object_id);
  void add_particle(int p, int obj_id, const Eigen::Vector3d &x,
                    const Eigen::Vector3d &v, double density, double pressure,
                    int material, int is_dynamic, const Eigen::Vector3i &color);
  void add_particles(int object_id, int new_particles_num,
                     const Eigen::MatrixXd &new_particles_positions,
                     const Eigen::MatrixXd &new_particles_velocity,
                     const Eigen::VectorXd &new_particle_density,
                     const Eigen::VectorXd &new_particle_pressure,
                     const Eigen::VectorXi &new_particles_material,
                     const Eigen::VectorXi &new_particles_is_dynamic,
                     const Eigen::MatrixXi &new_particles_color);
  void _add_particles(int object_id, int new_particles_num,
                      const Eigen::MatrixXd &new_particles_positions,
                      const Eigen::MatrixXd &new_particles_velocity,
                      const Eigen::VectorXd &new_particle_density,
                      const Eigen::VectorXd &new_particle_pressure,
                      const Eigen::VectorXi &new_particles_material,
                      const Eigen::VectorXi &new_particles_is_dynamic,
                      const Eigen::MatrixXi &new_particles_color);

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
  void add_cube(int object_id, const Eigen::Vector3d &lower_corner,
                const Eigen::Vector3d &cube_size,
                const Eigen::Vector3d &velocity, double density,
                bool is_dynamic, const Eigen::Vector3i &color, int material,
                double space);

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

  // Particle related properties
  Eigen::VectorXi particle_object_ids;
  Eigen::MatrixXd particle_positions;
  Eigen::MatrixXd particle_velocities;
  Eigen::MatrixXd particle_accelerations;
  Eigen::VectorXd particle_rest_volumes;
  Eigen::VectorXd particle_masses;
  Eigen::VectorXd particle_densities;
  Eigen::VectorXd particle_pressures;
  Eigen::VectorXi particle_materials;
  Eigen::MatrixXi particle_colors;
  Eigen::VectorXi particle_is_dynamic;

  Eigen::VectorXi object_materials;
  int object_num;

  Eigen::MatrixXd rigid_particle_original_positions;
  Eigen::VectorXi rigid_body_is_dynamic;
  Eigen::MatrixXd rigid_body_original_centers_of_mass;
  Eigen::VectorXd rigid_body_masses;
  Eigen::MatrixXd rigid_body_centers_of_mass;
  Eigen::MatrixXd rigid_body_rotations;
  Eigen::MatrixXd rigid_body_torques;
  Eigen::MatrixXd rigid_body_forces;
  Eigen::MatrixXd rigid_body_velocities;
  Eigen::MatrixXd rigid_body_angular_velocities;
  Eigen::VectorXi rigid_body_particle_num;

  // Buffer for sort
  Eigen::VectorXi particle_object_ids_buffer;
  Eigen::MatrixXd particle_positions_buffer;
  Eigen::MatrixXd rigid_particle_original_positions_buffer;
  Eigen::MatrixXd particle_velocities_buffer;
  Eigen::VectorXd particle_rest_volumes_buffer;
  Eigen::VectorXd particle_masses_buffer;
  Eigen::VectorXd particle_densities_buffer;
  Eigen::VectorXi particle_materials_buffer;
  Eigen::MatrixXi particle_colors_buffer;
  Eigen::VectorXi is_dynamic_buffer;

  // Visibility of object
  Eigen::VectorXi object_visibility;

  // Grid id for each particle
  Eigen::VectorXi grid_ids;
  Eigen::VectorXi grid_ids_buffer;
  Eigen::VectorXi grid_ids_new;

  Eigen::MatrixXd x_vis_buffer;
  Eigen::MatrixXd color_vis_buffer;
};

#endif // BASE_CONTAINER_H