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
}

void BaseContainer::object_initialize() {
  std::ifstream input("./data/models/dragon.obj");
  Polyhedron poly;
  if (input && (input >> poly)) {
    rigid_bodies.push_back(poly);
  } else {
    std::cerr << "Error loading mesh from file." << std::endl;
  }

  // 使用indicators库显示进度条
  indicators::ProgressBar bar{
      indicators::option::BarWidth{50},
      indicators::option::Start{"["},
      indicators::option::Fill{"="},
      indicators::option::Lead{">"},
      indicators::option::Remainder{" "},
      indicators::option::End{"]"},
      indicators::option::PostfixText{"Loading rigid bodies"},
      indicators::option::ForegroundColor{indicators::Color::green},
      indicators::option::ShowElapsedTime{true},
      indicators::option::ShowRemainingTime{true},
      indicators::option::MaxProgress{100}};

  for (int i = 0; i <= 100; ++i) {
    bar.set_progress(i);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // 使用std::accumulate替代reduce
  std::vector<int> numbers = {1, 2, 3, 4, 5};
  int sum = std::accumulate(numbers.begin(), numbers.end(), 0);
  std::cout << "Sum: " << sum << std::endl;

  // 使用std::bind替代partial
  auto add = [](int a, int b) { return a + b; };
  auto add_five = std::bind(add, 5, std::placeholders::_1);
  std::cout << "5 + 3 = " << add_five(3) << std::endl;

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