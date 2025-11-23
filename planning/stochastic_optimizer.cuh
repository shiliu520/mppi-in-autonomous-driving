/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:16:24
 * @LastEditTime: 2025-11-23 22:23:11
 * @FilePath: /mppi-in-autonomous-driving/planning/stochastic_optimizer.cuh
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/protos/planning_info.pb.h"
#include "common/reference_line.hpp"
#include "mppi/controllers/MPPI/mppi_controller.cuh"
#include "mppi/feedback_controllers/DDP/ddp.cuh"
#include "trajectory_cost.cuh"
#include "vehicle_dynamics.cuh"
#include <memory>
#include <Eigen/Core>

class StochasticOptimizer {
public:
  StochasticOptimizer(/* args */);
  ~StochasticOptimizer();

  ControlInput plan_once(const StateInfo& _current_state, const std::shared_ptr<ReferenceLine>& reference_line);
  Eigen::MatrixXf get_optimized_trajectory() const;
  planning::protos::PlanningInfo get_debug_result(const StateInfo& current_state) const;

private:
  using SAMPLER_T =
      mppi::sampling_distributions::GaussianDistribution<VehicleDynamics::DYN_PARAMS_T>;

  double delta_time_ = 0.1;
  VehicleDynamics* dynamics_ = nullptr;
  TrajectoryCost* trajectory_cost_ = nullptr;
  SAMPLER_T* sampler_ = nullptr;
  DDPFeedback<VehicleDynamics, 64>* ddp_feedback_ = nullptr;
  VanillaMPPIController<VehicleDynamics, TrajectoryCost, DDPFeedback<VehicleDynamics, 64>, 64,
                        8192>* mppi_controller_ = nullptr;
  
  float target_accel_ = 0.0;
  float target_steer_ = 0.0;
  float cruise_velocity_ = 10.0;
  float cost_time_ms_ = 0.0;
};
