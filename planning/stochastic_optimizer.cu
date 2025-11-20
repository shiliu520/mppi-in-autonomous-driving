/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:39:47
 * @LastEditTime: 2025-11-20 23:53:32
 * @FilePath: /mppi-in-autonomous-driving/planning/stochastic_optimizer.cu
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "stochastic_optimizer.cuh"

StochasticOptimizer::StochasticOptimizer(/* args */) {
  dynamics_ = new VehicleDynamics(2.8);
  dynamics_->control_rngs_[0].x = -2;    // min jerk
  dynamics_->control_rngs_[0].y = 2;     // max jerk
  dynamics_->control_rngs_[1].x = -0.2;  // min steering rate
  dynamics_->control_rngs_[1].y = 0.2;   // max steering rate

  trajectory_cost_ = new TrajectoryCost;
  TrajectoryCostParams new_params;
  new_params.target_velocity = 10.0f;
  new_params.bike_position_coeff = 50.0f;
  new_params.bike_velocity_coeff = 10.0f;
  new_params.bike_angle_coeff = 100.0f;
  new_params.accel_effort_coeff = 5.0f;
  new_params.steer_effort_coeff = 100.0f;
  trajectory_cost_->setParams(new_params);

  float delta_time_ = 0.1;
  int max_iter = 1;
  float lambda = 2.0;
  float alpha = 0.0;
  const int num_timesteps = 64;

  auto sampler_params = SAMPLER_T::SAMPLING_PARAMS_T();
  sampler_params.std_dev[0] = 0.5;
  sampler_params.std_dev[1] = 0.05;
  sampler_ = new SAMPLER_T(sampler_params);

  ddp_feedback_ = new DDPFeedback<VehicleDynamics, num_timesteps>(dynamics_, delta_time_);

  mppi_controller_ = new VanillaMPPIController<VehicleDynamics, TrajectoryCost,
                                               DDPFeedback<VehicleDynamics, 64>, 64, 8192>(
      dynamics_, trajectory_cost_, ddp_feedback_, sampler_, delta_time_, max_iter, lambda, alpha);
  auto controller_params = mppi_controller_->getParams();
  controller_params.dynamics_rollout_dim_ = dim3(64, 1, 1);
  controller_params.cost_rollout_dim_ = dim3(64, 1, 1);
  mppi_controller_->setParams(controller_params);
}

StochasticOptimizer::~StochasticOptimizer() {
  delete dynamics_;
  delete trajectory_cost_;
  delete sampler_;
  delete ddp_feedback_;
  delete mppi_controller_;
}

ControlInput StochasticOptimizer::plan_once(const StateInfo& _current_state) {
  VehicleDynamics::state_array cur_state;
  VehicleDynamics::state_array next_state = dynamics_->getZeroState();
  VehicleDynamics::state_array xdot = dynamics_->getZeroState();
  VehicleDynamics::output_array output = VehicleDynamics::output_array::Zero();
  cur_state(0) = _current_state.x;
  cur_state(1) = _current_state.y;
  cur_state(2) = _current_state.velocity;
  cur_state(3) = _current_state.yaw;
  cur_state(4) = _current_state.accel;
  cur_state(5) = _current_state.steer;

  mppi_controller_->computeControl(cur_state, 1);
  VehicleDynamics::control_array control =
      mppi_controller_->getControlSeq().block(0, 0, VehicleDynamics::CONTROL_DIM, 1);
  dynamics_->enforceConstraints(cur_state, control);
  dynamics_->step(cur_state, next_state, xdot, control, output, 0, delta_time_);
  mppi_controller_->slideControlSequence(1);

  return {next_state(4), next_state(5)};
}

Eigen::MatrixXf StochasticOptimizer::get_optimized_trajectory() const {
  return mppi_controller_->getTargetStateSeq();
}