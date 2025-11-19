/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:16:24
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-11-19 23:54:52
 * @FilePath: /mppi-in-autonomous-driving/planning/stochastic_optimizer.cuh
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "mppi/controllers/MPPI/mppi_controller.cuh"
#include "mppi/feedback_controllers/DDP/ddp.cuh"
#include "trajectory_cost.cuh"
#include "vehicle_dynamics.cuh"

class StochasticOptimizer {
public:
  StochasticOptimizer(/* args */);
  ~StochasticOptimizer();

  ControlInput plan_once(const StateInfo& _current_state);

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
};
