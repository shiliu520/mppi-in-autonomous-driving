#include "common/common.hpp"

// protos::config::SimulationConfig LoadSimulationConfigFromYAML(const YAML::Node& config) {
//   protos::config::SimulationConfig sim_config;
//   if (config["vehicle_info"]) {
//     auto vehicle_info_ptr = sim_config.mutable_vehicle_info();
//     auto vehicle_info_node = config["vehicle_info"];
//     vehicle_info_ptr->set_wheel_base_m(vehicle_info_node["wheel_base_m"].as<float>(3.0));
//     vehicle_info_ptr->set_vehicle_width_m(vehicle_info_node["vehicle_width_m"].as<float>(1.85));
//     vehicle_info_ptr->set_vehicle_length_m(vehicle_info_node["vehicle_length_m"].as<float>(4.85));
//     vehicle_info_ptr->set_axle_to_front_bumper_m(vehicle_info_node["axle_to_front_bumper_m"].as<float>(3.90));
//     vehicle_info_ptr->set_axle_to_rear_bumper_m(vehicle_info_node["axle_to_rear_bumper_m"].as<float>(0.95));
//   }

//   if (config["planning"]) {
//     auto planning_params_ptr = sim_config.mutable_planning_parameters();
//     auto planning_node = config["planning"];
//     planning_params_ptr->set_desired_speed(planning_node["desired_speed"].as<double>(15.0));
//     planning_params_ptr->set_max_accel_mps2(planning_node["max_accel_mps2"].as<double>(2.0));
//     planning_params_ptr->set_max_decel_mps2(planning_node["max_decel_mps2"].as<double>(-3.0));
//     planning_params_ptr->set_max_steer_rad(planning_node["max_steer_rad"].as<double>(0.6));
//     planning_params_ptr->set_max_steer_rate_rad_per_sec(planning_node["max_steer_rate_rad_per_sec"].as<double>(0.5));
//   }

  // if (config["simulation"]) {
  //   auto sim_node = config["simulation"];
  //   if (sim_node["time_step_sec"]) {
  //     sim_config.set_time_step_sec(sim_node["time_step_sec"].as<double>(0.1));
  //   }
  //   if (sim_node["horizon_length"]) {
  //     sim_config.set_horizon_length(sim_node["horizon_length"].as<int>(64));
  //   }
  //   if (sim_node["perception_range_m"]) {
  //     sim_config.set_perception_range_m(sim_node["perception_range_m"].as<double>(100.0));
  //   }
  // }
//   return sim_config;
// }