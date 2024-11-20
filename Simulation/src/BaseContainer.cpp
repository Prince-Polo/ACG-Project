#include "BaseContainer.h"
#include <cassert>
#include <cmath>
#include <fstream>
#include <functional>
#include <indicators/progress_bar.hpp>
#include <iostream>
#include <numeric>

BaseContainer::BaseContainer(SimConfig &config, bool GGUI)
    : cfg(config), GGUI(GGUI), total_time(0.0) {
  domain_start = Eigen::Vector3d(0.0, 0.0, 0.0);
  domain_start = cfg.get_vec3_cfg("domainStart");

  assert(domain_start.y() >= 0.0 && "domain start y should be greater than 0");

  domain_end = Eigen::Vector3d(1.0, 1.0, 1.0);
  domain_end = cfg.get_vec3_cfg("domainEnd");

  domain_size = domain_end - domain_start;

  dim = 3; // Assuming 3D for simplicity
  std::cout << "Dimension: " << dim << std::endl;

  // material type
  material_rigid = 2;
  material_fluid = 1;

  // particle radius
  dx = 0.01;
  dx = cfg.get_double_cfg("particleRadius");

  particle_diameter = 2 * dx;
  dh = dx * 4;

  particle_spacing = particle_diameter;
  if (cfg.get_double_cfg("particleSpacing")) {
    particle_spacing = cfg.get_double_cfg("particleSpacing");
  }

  // initial volume fraction
  V0 = 0.8 * std::pow(particle_diameter, dim);
  particle_num = 0;

  max_num_object = 20;

  grid_size = dh;
  grid_num = (domain_size / grid_size).array().ceil().cast<int>();
  std::cout << "grid size: " << grid_num.transpose() << std::endl;
  padding = grid_size;

  add_domain_box = cfg.get_bool_cfg("addDomainBox");
  if (add_domain_box) {
    domain_box_start =
        domain_start + Eigen::Vector3d(padding, padding, padding);
    domain_box_size =
        domain_size - 2.0 * Eigen::Vector3d(padding, padding, padding);
    domain_box_thickness = 0.03;
  } else {
    domain_box_thickness = 0.0;
  }

  object_initialize();

  //========== Allocate memory ==========//
  // Particle num of each grid
  int num_grid = grid_num.prod(); // handle 3D together

  particle_object_ids.resize(particle_max_num);
  particle_positions.resize(particle_max_num, dim);
  particle_velocities.resize(particle_max_num, dim);
  particle_accelerations.resize(particle_max_num, dim);
  particle_rest_volumes.resize(particle_max_num);
  particle_masses.resize(particle_max_num);
  particle_densities.resize(particle_max_num);
  particle_pressures.resize(particle_max_num);
  particle_materials.resize(particle_max_num);
  particle_colors.resize(particle_max_num, 3);
  particle_is_dynamic.resize(particle_max_num);

  object_materials.resize(max_num_object);

  object_num = num_fluid_object + num_rigid_object +
               (add_domain_box ? 1 : 0); // add 1 for domain box object

  rigid_particle_original_positions.resize(particle_max_num, dim);
  rigid_body_is_dynamic.resize(max_num_object);
  rigid_body_original_centers_of_mass.resize(max_num_object, dim);
  rigid_body_masses.resize(max_num_object);
  rigid_body_centers_of_mass.resize(max_num_object, dim);
  rigid_body_rotations.resize(max_num_object, dim * dim);
  rigid_body_torques.resize(max_num_object, dim);
  rigid_body_forces.resize(max_num_object, dim);
  rigid_body_velocities.resize(max_num_object, dim);
  rigid_body_angular_velocities.resize(max_num_object, dim);
  rigid_body_particle_num.resize(max_num_object);

  // Buffer for sort
  particle_object_ids_buffer.resize(particle_max_num);
  particle_positions_buffer.resize(particle_max_num, dim);
  rigid_particle_original_positions_buffer.resize(particle_max_num, dim);
  particle_velocities_buffer.resize(particle_max_num, dim);
  particle_rest_volumes_buffer.resize(particle_max_num);
  particle_masses_buffer.resize(particle_max_num);
  particle_densities_buffer.resize(particle_max_num);
  particle_materials_buffer.resize(particle_max_num);
  particle_colors_buffer.resize(particle_max_num, 3);
  is_dynamic_buffer.resize(particle_max_num);

  // Visibility of object
  object_visibility.resize(max_num_object);

  // Grid id for each particle
  grid_ids.resize(particle_max_num);
  grid_ids_buffer.resize(particle_max_num);
  grid_ids_new.resize(particle_max_num);

  if (GGUI) {
    x_vis_buffer.resize(particle_max_num, dim);
    color_vis_buffer.resize(particle_max_num, 3);
  }

  if (add_domain_box) {
    // Add domain box
    add_box(object_num - 1, domain_box_start, domain_box_size,
            domain_box_thickness, material_rigid, false, particle_spacing,
            Eigen::Vector3i(127, 127, 127));

    object_visibility[object_num - 1] = 0;
    object_materials[object_num - 1] = material_rigid;
    rigid_body_is_dynamic[object_num - 1] = 0;
    rigid_body_velocities.row(object_num - 1).setZero();
    object_collection[object_num - 1] =
        std::unordered_map<std::string, nlohmann::json>();
  }
}

void BaseContainer::object_initialize() {
  //========== Compute number of particles ==========//
  fluid_particle_num = 0;
  rigid_body_particle_num = 0;

  // Process Fluid Bodies from Mesh
  fluid_bodies = cfg.get_fluid_bodies();
  for (auto &fluid_body : fluid_bodies) {
    Eigen::MatrixXd voxelized_points_np =
        load_fluid_body(fluid_body, particle_spacing);
    fluid_body["particleNum"] = voxelized_points_np.rows();
    fluid_body["voxelizedPoints"] = voxelized_points_np;
    fluid_particle_num += voxelized_points_np.rows();
  }

  // Process Fluid Blocks
  fluid_blocks = cfg.get_fluid_blocks();
  for (auto &fluid : fluid_blocks) {
    int particle_num = compute_cube_particle_num(fluid["start"], fluid["end"],
                                                 particle_spacing);
    fluid["particleNum"] = particle_num;
    fluid_particle_num += particle_num;
  }

  int num_fluid_object = fluid_blocks.size() + fluid_bodies.size();

  // Process Rigid Bodies from Mesh
  rigid_bodies = cfg.get_rigid_bodies();
  for (auto &rigid_body : rigid_bodies) {
    Eigen::MatrixXd voxelized_points_np =
        load_rigid_body(rigid_body, particle_spacing);
    rigid_body["particleNum"] = voxelized_points_np.rows();
    rigid_body["voxelizedPoints"] = voxelized_points_np;
    rigid_body_particle_num += voxelized_points_np.rows();
  }

  // Process Rigid Blocks
  rigid_blocks = cfg.get_rigid_blocks();
  for (auto &rigid_block : rigid_blocks) {
    // Not implemented
  }

  int num_rigid_object = rigid_blocks.size() + rigid_bodies.size();
  std::cout << "Number of rigid bodies and rigid blocks: " << num_rigid_object
            << std::endl;

  fluid_particle_num = fluid_particle_num;
  rigid_body_particle_num = rigid_body_particle_num;
  particle_max_num =
      fluid_particle_num + rigid_body_particle_num +
      (add_domain_box
           ? compute_box_particle_num(domain_box_start, domain_box_size,
                                      particle_spacing, domain_box_thickness)
           : 0);

  std::cout << "Fluid particle num: " << fluid_particle_num
            << ", Rigid body particle num: " << rigid_body_particle_num
            << std::endl;
}

Eigen::MatrixXd BaseContainer::load_fluid_body(
    const std::unordered_map<std::string, nlohmann::json> &fluid_body,
    double pitch) {
  // Implement the function to load fluid body
  // This is a placeholder implementation
  return Eigen::MatrixXd::Zero(0, 3);
}

Eigen::MatrixXd BaseContainer::load_rigid_body(
    const std::unordered_map<std::string, nlohmann::json> &rigid_body,
    double pitch) {
  // Implement the function to load rigid body
  // This is a placeholder implementation
  return Eigen::MatrixXd::Zero(0, 3);
}

int BaseContainer::compute_cube_particle_num(const Eigen::Vector3d &start,
                                             const Eigen::Vector3d &end,
                                             double space) {
  // Implement the function to compute the number of particles in a cube
  // This is a placeholder implementation
  return 0;
}

int BaseContainer::compute_box_particle_num(const Eigen::Vector3d &lower_corner,
                                            const Eigen::Vector3d &cube_size,
                                            double space, double thickness) {
  // Implement the function to compute the number of particles in a box
  // This is a placeholder implementation
  return 0;
}

void BaseContainer::insert_object() {
  // Fluid block
  for (auto &fluid : fluid_blocks) {
    int obj_id = fluid["objectId"];

    if (std::find(present_object.begin(), present_object.end(), obj_id) !=
        present_object.end())
      continue;
    if (fluid["entryTime"] > total_time)
      continue;

    Eigen::Vector3d offset = fluid["translation"];
    Eigen::Vector3d start = fluid["start"] + offset;
    Eigen::Vector3d end = fluid["end"] + offset;
    Eigen::Vector3d scale = fluid["scale"];
    Eigen::Vector3d velocity = fluid["velocity"];
    double density = fluid["density"];
    Eigen::Vector3i color = fluid["color"];
    object_id_fluid_body.insert(obj_id);

    if (fluid.contains("visible")) {
      object_visibility[obj_id] = fluid["visible"];
    } else {
      object_visibility[obj_id] = 1;
    }

    object_materials[obj_id] = material_fluid;
    object_collection[obj_id] = fluid;

    add_cube(obj_id, start, (end - start) * scale, velocity, density, true,
             color, material_fluid, particle_spacing);

    present_object.push_back(obj_id);
  }

  // Fluid body
  for (auto &fluid_body : fluid_bodies) {
    int obj_id = fluid_body["objectId"];

    if (std::find(present_object.begin(), present_object.end(), obj_id) !=
        present_object.end())
      continue;
    if (fluid_body["entryTime"] > total_time)
      continue;

    int num_particles_obj = fluid_body["particleNum"];
    Eigen::MatrixXd voxelized_points_np = fluid_body["voxelizedPoints"];
    Eigen::Vector3d velocity = fluid_body["velocity"];
    double density = fluid_body["density"];
    Eigen::Vector3i color = fluid_body["color"];

    if (fluid_body.contains("visible")) {
      object_visibility[obj_id] = fluid_body["visible"];
    } else {
      object_visibility[obj_id] = 1;
    }

    object_materials[obj_id] = material_fluid;
    object_id_fluid_body.insert(obj_id);
    object_collection[obj_id] = fluid_body;

    add_particles(obj_id, num_particles_obj, voxelized_points_np,
                  Eigen::MatrixXd::Constant(num_particles_obj, dim, velocity),
                  Eigen::VectorXd::Constant(num_particles_obj, density),
                  Eigen::VectorXd::Zero(num_particles_obj),
                  Eigen::VectorXi::Constant(num_particles_obj, material_fluid),
                  Eigen::VectorXi::Constant(num_particles_obj, 1),
                  Eigen::MatrixXi::Constant(num_particles_obj, 3, color));

    present_object.push_back(obj_id);
    fluid_particle_num += num_particles_obj;
  }

  // Rigid body
  for (auto &rigid_body : rigid_bodies) {
    int obj_id = rigid_body["objectId"];

    if (std::find(present_object.begin(), present_object.end(), obj_id) !=
        present_object.end())
      continue;
    if (rigid_body["entryTime"] > total_time)
      continue;

    object_id_rigid_body.insert(obj_id);
    int num_particles_obj = rigid_body["particleNum"];
    rigid_body_particle_num[obj_id] = num_particles_obj;
    Eigen::MatrixXd voxelized_points_np = rigid_body["voxelizedPoints"];
    bool is_dynamic = rigid_body["isDynamic"];
    Eigen::Vector3d velocity;
    if (is_dynamic) {
      velocity = rigid_body["velocity"];
    } else {
      velocity = Eigen::Vector3d::Zero(dim);
    }
    double density = rigid_body["density"];
    Eigen::Vector3i color = rigid_body["color"];

    if (rigid_body.contains("visible")) {
      object_visibility[obj_id] = rigid_body["visible"];
    } else {
      object_visibility[obj_id] = 1;
    }

    object_materials[obj_id] = material_rigid;
    object_collection[obj_id] = rigid_body;

    add_particles(obj_id, num_particles_obj, voxelized_points_np,
                  Eigen::MatrixXd::Constant(num_particles_obj, dim, velocity),
                  Eigen::VectorXd::Constant(num_particles_obj, density),
                  Eigen::VectorXd::Zero(num_particles_obj),
                  Eigen::VectorXi::Constant(num_particles_obj, material_rigid),
                  Eigen::VectorXi::Constant(num_particles_obj, is_dynamic),
                  Eigen::MatrixXi::Constant(num_particles_obj, 3, color));

    rigid_body_is_dynamic[obj_id] = is_dynamic;
    rigid_body_velocities.row(obj_id) = velocity;

    if (is_dynamic) {
      rigid_body_masses[obj_id] = compute_rigid_body_mass(obj_id);
      rigid_body_is_dynamic[obj_id] = 1;
      // rigid_com = compute_rigid_body_center_of_mass(obj_id);
      // ! here we assume the center of mass is exactly the base frame center
      // and calculated it in the bullet solver.
    }
    present_object.push_back(obj_id);
  }
}

void BaseContainer::add_cube(int object_id, const Eigen::Vector3d &lower_corner,
                             const Eigen::Vector3d &cube_size,
                             const Eigen::Vector3d &velocity, double density,
                             bool is_dynamic, const Eigen::Vector3i &color,
                             int material, double space) {
  // Implement the function to add particles in a cube
  // This is a placeholder implementation
}

void BaseContainer::add_particle(int p, int obj_id, const Eigen::Vector3d &x,
                                 const Eigen::Vector3d &v, double density,
                                 double pressure, int material, int is_dynamic,
                                 const Eigen::Vector3i &color) {
  particle_object_ids[p] = obj_id;
  particle_positions.row(p) = x;
  rigid_particle_original_positions.row(p) = x;
  particle_velocities.row(p) = v;
  particle_densities[p] = density;
  particle_rest_volumes[p] = V0;
  particle_masses[p] = V0 * density;
  particle_pressures[p] = pressure;
  particle_materials[p] = material;
  particle_is_dynamic[p] = is_dynamic;
  particle_colors.row(p) = color;
}

void BaseContainer::add_particles(
    int object_id, int new_particles_num, const Eigen::MatrixXd &positions,
    const Eigen::MatrixXd &velocities, const Eigen::VectorXd &densities,
    const Eigen::VectorXd &pressures, const Eigen::VectorXi &materials,
    const Eigen::VectorXi &is_dynamic, const Eigen::MatrixXi &colors) {
  for (int p = particle_num; p < particle_num + new_particles_num; ++p) {
    Eigen::Vector3d v = Eigen::Vector3d::Zero(dim);
    Eigen::Vector3d x = Eigen::Vector3d::Zero(dim);
    for (int d = 0; d < dim; ++d) {
      v[d] = velocities(p - particle_num, d);
      x[d] = positions(p - particle_num, d);
    }
    add_particle(p, object_id, x, v, densities[p - particle_num],
                 pressures[p - particle_num], materials[p - particle_num],
                 is_dynamic[p - particle_num], colors.row(p - particle_num));
  }
  particle_num += new_particles_num;
}

double BaseContainer::compute_rigid_body_mass(int object_id) {
  double sum_m = 0.0;
  for (int p_i = 0; p_i < particle_num; ++p_i) {
    if (particle_object_ids[p_i] == object_id && particle_is_dynamic[p_i]) {
      sum_m += particle_densities[p_i] * V0;
    }
  }
  return sum_m;
}