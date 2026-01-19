/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:57:28
 * @LastEditTime: 2026-01-20 00:42:53
 * @FilePath: /mppi-in-autonomous-driving/modules/simulator/simulator.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/reference_line.hpp"
#include "commonroad_cpp/world.h"
#include "modules/visualizer/visualizer.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

class Simulator {
public:
  Simulator() = delete;
  Simulator(const YAML::Node& config, std::shared_ptr<Visualizer> visualizer = nullptr);
  ~Simulator();

  bool start();  // start the simulation thread
  void stop();   // stop the simulation thread

  StateInfo get_ego_state() const;
  void set_ego_control_input(const ControlInput& input);
  std::shared_ptr<ReferenceLine> get_reference_line(void) const;
  std::shared_ptr<common::ObstacleList> get_obstacle_list(void);

private:
  void simulation_loop(void);
  void update_ego_state(void);

private:
  double perception_range_m_ = 100.0;
  double last_ego_update_time_ = 0.0;
  StateInfo ego_state_;
  VehicleInfo vehicle_info_;
  planning::protos::PlanningInfo planning_info_;
  std::shared_ptr<ReferenceLine> routing_reference_line_{nullptr};
  std::shared_ptr<ReferenceLine> reference_line_{nullptr};
  std::shared_ptr<spdlog::logger> logger_{nullptr};
  YAML::Node config_{};

  std::thread sim_thread_;
  std::atomic<bool> running_{false};
  mutable std::shared_mutex ego_state_mutex_;
  mutable std::shared_mutex reference_line_mutex_;
  mutable std::shared_mutex obstacle_prediction_mutex_;
  std::unordered_map<std::string, std::vector<PathPoint>> obstacle_predictions_{};

  std::atomic<size_t> sim_world_timestep_{0};
  std::unique_ptr<World> sim_world_{nullptr};
  std::shared_ptr<Visualizer> visualizer_{nullptr};
};
