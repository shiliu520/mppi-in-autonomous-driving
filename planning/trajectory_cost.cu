/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:30:09
 * @LastEditTime: 2025-11-22 23:21:40
 * @FilePath: /mppi-in-autonomous-driving/planning/trajectory_cost.cu
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "trajectory_cost.cuh"

#include <cmath>

TrajectoryCost::TrajectoryCost(cudaStream_t stream) { bindToStream(stream); }

float TrajectoryCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep,
                                       int* crash_status) {
  float position_cost = params_.bike_position_coeff * std::abs(s(1) - 3.2);
  float velocity_cost = params_.bike_velocity_coeff * std::abs(s(2) - params_.target_velocity);
  float angle_cost = params_.bike_angle_coeff * std::abs(s(3));
  float accel_cost = params_.accel_effort_coeff * std::abs(s(4));
  float steer_cost = params_.steer_effort_coeff * std::abs(s(5));
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost;
}

float TrajectoryCost::terminalCost(const Eigen::Ref<const output_array> s) {
  float position_cost = params_.bike_position_coeff * std::abs(s(1) - 3.2);
  float velocity_cost = params_.bike_velocity_coeff * std::abs(s(2) - params_.target_velocity);
  float angle_cost = params_.bike_angle_coeff * std::abs(s(3));
  float accel_cost = params_.accel_effort_coeff * std::abs(s(4));
  float steer_cost = params_.steer_effort_coeff * std::abs(s(5));
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost;
}
