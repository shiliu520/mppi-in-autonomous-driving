/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:39:47
 * @LastEditTime: 2025-12-09 23:10:02
 * @FilePath: /mppi-in-autonomous-driving/planning/stochastic_optimizer.cu
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "stochastic_optimizer.cuh"

#include <spdlog/sinks/stdout_color_sinks.h>

template<int NUM_ROLLOUTS>
StochasticOptimizer<NUM_ROLLOUTS>::StochasticOptimizer(const YAML::Node& config) {
  auto planning_config = config["planning"];
  auto constraint_limits = planning_config["constraint_limits"];
  auto cost_weights = planning_config["cost_weights"];
  auto mppi_config = planning_config["mppi_params"];
  auto vehicle_info = config["vehicle_info"];

  std::string planning_log_level = planning_config["log_level"].as<std::string>("info");
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_ = std::make_shared<spdlog::logger>("stop_line_logger", console_sink);
  logger_->set_level(spdlog::level::from_str(planning_log_level));
  logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^\033[1m%l\033[0m%$] [%s:%#] %v");

  dynamics_ = new VehicleDynamics(2.8);
  dynamics_->control_rngs_[0].x = constraint_limits["min_jerk_mps3"].as<float>(-1.5);
  dynamics_->control_rngs_[0].y = constraint_limits["max_jerk_mps3"].as<float>(1.5);
  dynamics_->control_rngs_[1].x = -constraint_limits["max_steering_rate_rps"].as<float>(0.07);
  dynamics_->control_rngs_[1].y = constraint_limits["max_steering_rate_rps"].as<float>(0.07);

  cruise_velocity_ = planning_config["desired_speed"].as<float>(16.0f);
  trajectory_cost_ = new TrajectoryCost;
  TrajectoryCostParams new_params;
  new_params.target_velocity = cruise_velocity_;
  new_params.position_coeff = cost_weights["position_weight"].as<float>(15.0f);
  new_params.velocity_coeff = cost_weights["velocity_weight"].as<float>(10.0f);
  new_params.angle_coeff = cost_weights["heading_weight"].as<float>(75.0f);
  new_params.accel_effort_coeff = cost_weights["accel_effort_weight"].as<float>(10.0f);
  new_params.steer_effort_coeff = cost_weights["steer_effort_weight"].as<float>(80.0f);
  new_params.max_accel = constraint_limits["max_accel_mps2"].as<float>(2.0f);
  new_params.min_accel = constraint_limits["min_accel_mps2"].as<float>(-4.0f);
  new_params.max_steer_angle = constraint_limits["max_steer_angle_rad"].as<float>(0.15f);
  new_params.min_steer_angle = -constraint_limits["max_steer_angle_rad"].as<float>(0.15f);
  new_params.control_cost_coeff[0] = cost_weights["jerk_effort_weight"].as<float>(20.0f);
  new_params.control_cost_coeff[1] = cost_weights["steer_effort_weight"].as<float>(120.0f);
  new_params.longitudinal_safety_margin =
      constraint_limits["longitudinal_safety_margin_m"].as<float>(3.0f);
  new_params.lateral_safety_margin = constraint_limits["lateral_safety_margin_m"].as<float>(0.6f);
  new_params.wheelbase = vehicle_info["wheel_base_m"].as<float>(3.0f);
  new_params.vehicle_width = vehicle_info["vehicle_width_m"].as<float>(1.85f);
  new_params.vehicle_length = vehicle_info["vehicle_length_m"].as<float>(4.85f);
  new_params.axle_to_front_bumper = vehicle_info["axle_to_front_bumper_m"].as<float>(3.90f);
  new_params.axle_to_rear_bumper = vehicle_info["axle_to_rear_bumper_m"].as<float>(0.95f);
  trajectory_cost_->setParams(new_params);

  float delta_time_ = 0.1;
  int max_iter = 1;
  float lambda = mppi_config["lambda"].as<float>(2.5);
  float alpha = 0.0;

  auto sampler_params = SAMPLER_T::SAMPLING_PARAMS_T();
  sampler_params.std_dev[0] = mppi_config["noise_sigma"]["jerk_std"].as<float>(0.5);
  sampler_params.std_dev[1] = mppi_config["noise_sigma"]["steer_rate_std"].as<float>(0.02);
  sampler_ = new SAMPLER_T(sampler_params);

  ddp_feedback_ = new DDPFeedback<VehicleDynamics, kHorizonLength>(dynamics_, delta_time_);
  mppi_controller_ = new VanillaMPPIController<VehicleDynamics, TrajectoryCost,
                                               DDPFeedback<VehicleDynamics, kHorizonLength>,
                                               kHorizonLength, NUM_ROLLOUTS>(
      dynamics_, trajectory_cost_, ddp_feedback_, sampler_, delta_time_, max_iter, lambda, alpha);
  auto controller_params = mppi_controller_->getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  mppi_controller_->setParams(controller_params);
  // mppi_controller_->setPercentageSampledControlTrajectoriesHelper(0.004);
  // mppi_controller_->setPercentageSampledControlTrajectoriesHelper(0.0079);
  mppi_controller_->setTopNSampledControlTrajectoriesHelper(128);
}

template<int NUM_ROLLOUTS>
StochasticOptimizer<NUM_ROLLOUTS>::~StochasticOptimizer() {
  delete dynamics_;
  delete trajectory_cost_;
  delete sampler_;
  delete ddp_feedback_;
  delete mppi_controller_;
}

template<int NUM_ROLLOUTS>
ControlInput StochasticOptimizer<NUM_ROLLOUTS>::plan_once(
    const StateInfo& _current_state, const std::shared_ptr<ReferenceLine>& reference_line,
    const std::shared_ptr<common::ObstacleList>& obstacle_list) {
  // Update waypoints in trajectory cost function
  if (reference_line) {
    trajectory_cost_->setWaypoints(reference_line);
    // Initialize matched index cache with current vehicle position
    trajectory_cost_->updateMatchedIndex(_current_state.x, _current_state.y);
  }

  // Update obstacles in trajectory cost function
  if (obstacle_list) {
    trajectory_cost_->setObstacles(obstacle_list);
  }

  VehicleDynamics::state_array cur_state;
  VehicleDynamics::state_array next_state = dynamics_->getZeroState();
  VehicleDynamics::state_array xdot = dynamics_->getZeroState();
  VehicleDynamics::output_array output = VehicleDynamics::output_array::Zero();
  cur_state(0) = _current_state.x;
  cur_state(1) = _current_state.y;
  cur_state(2) = _current_state.velocity;
  cur_state(3) = _current_state.heading;
  cur_state(4) = _current_state.accel;
  cur_state(5) = _current_state.steer;

  TicToc solve_time_tic;
  mppi_controller_->computeControl(cur_state, 1);
  VehicleDynamics::control_array control =
      mppi_controller_->getControlSeq().block(0, 0, VehicleDynamics::CONTROL_DIM, 1);
  dynamics_->enforceConstraints(cur_state, control);
  dynamics_->step(cur_state, next_state, xdot, control, output, 0, delta_time_);
  mppi_controller_->slideControlSequence(1);
  cost_time_ms_ = solve_time_tic.toc() * 1000.0;
  LOG_DEBUG(logger_, "StochasticOptimizer plan_once cost time: {:.2f} ms", cost_time_ms_);

  target_accel_ = next_state(4);
  target_steer_ = next_state(5);

  return {target_accel_, target_steer_};
}

template<int NUM_ROLLOUTS>
Eigen::MatrixXf StochasticOptimizer<NUM_ROLLOUTS>::get_optimized_trajectory() const {
  return mppi_controller_->getTargetStateSeq();
}

template<int NUM_ROLLOUTS>
planning::protos::PlanningInfo StochasticOptimizer<NUM_ROLLOUTS>::get_debug_result(
    const StateInfo& current_state) const {
  TicToc get_debug_result_tic;
  planning::protos::PlanningInfo planning_info;
  planning_info.set_target_accel(target_accel_);
  planning_info.set_target_steer(target_steer_);
  planning_info.set_cruise_velocity(cruise_velocity_);
  planning_info.set_cost_time_ms(cost_time_ms_);
  planning_info.set_optimal_cost(mppi_controller_->getBaselineCost());

  // Set current state
  auto* state_proto = planning_info.mutable_current_state();
  state_proto->set_pos_x(current_state.x);
  state_proto->set_pos_y(current_state.y);
  state_proto->set_velocity(current_state.velocity);
  state_proto->set_heading(current_state.heading);
  state_proto->set_accel(current_state.accel);
  state_proto->set_steer(current_state.steer);

  // Set optimized trajectory
  Eigen::MatrixXf optimized_trajectory = mppi_controller_->getTargetStateSeq();
  for (int t = 0; t < optimized_trajectory.cols(); ++t) {
    auto* traj_point = planning_info.add_optimal_trajectory();
    traj_point->set_pos_x(optimized_trajectory(0, t));
    traj_point->set_pos_y(optimized_trajectory(1, t));
    traj_point->set_velocity(optimized_trajectory(2, t));
    traj_point->set_heading(optimized_trajectory(3, t));
    traj_point->set_accel(optimized_trajectory(4, t));
    traj_point->set_steer(optimized_trajectory(5, t));
  }
  // Set optimal control sequence
  Eigen::MatrixXf optimal_control_seq = mppi_controller_->getControlSeq();
  for (int t = 0; t < optimal_control_seq.cols(); ++t) {
    auto* control_proto = planning_info.add_optimal_control_seq();
    control_proto->set_jerk(optimal_control_seq(0, t));
    control_proto->set_steer_rate(optimal_control_seq(1, t));
  }

  // This step is a bit time-consuming and is only opened when debugging and visualizing samples.
  TicToc calculate_sample_tic;
  mppi_controller_->calculateSampledStateTrajectories();
  auto sampled_trajectories = mppi_controller_->getSampledOutputTrajectories();
  auto sampled_costs = mppi_controller_->getSampledCostTrajectories();
  LOG_DEBUG(logger_, "StochasticOptimizer calculate sampled trajectories cost time: {:.2f} ms",
            calculate_sample_tic.toc() * 1000.0);

  for (const auto& traj : sampled_trajectories) {
    auto* state_seq_proto = planning_info.add_sample_xs();
    for (int t = 0; t < traj.cols(); t += 4) {
      // downsample by 4 for brevity
      auto* traj_point = state_seq_proto->add_trajectory();
      traj_point->set_pos_x(traj(0, t));
      traj_point->set_pos_y(traj(1, t));
      traj_point->set_velocity(traj(2, t));
      traj_point->set_heading(traj(3, t));
      traj_point->set_accel(traj(4, t));
      traj_point->set_steer(traj(5, t));
    }
  }

  for (int i = 0; i < sampled_costs.size() && i < planning_info.sample_xs_size(); ++i) {
    planning_info.mutable_sample_xs(i)->set_cost(sampled_costs[i].col(0).sum());
  }

  LOG_DEBUG(logger_, "StochasticOptimizer get_debug_result cost time: {:.2f} ms",
            get_debug_result_tic.toc() * 1000.0);

  return planning_info;
}

template class StochasticOptimizer<1024>;
template class StochasticOptimizer<2048>;
template class StochasticOptimizer<4096>;
template class StochasticOptimizer<8192>;
template class StochasticOptimizer<16384>;
