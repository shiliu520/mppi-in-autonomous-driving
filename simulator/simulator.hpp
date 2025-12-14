/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:57:28
 * @LastEditTime: 2025-12-13 22:12:25
 * @FilePath: /mppi-in-autonomous-driving/simulator/simulator.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/protos/planning_info.pb.h"
#include "common/reference_line.hpp"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/world.h"
#include "foxglove/foxglove.hpp"
#include "foxglove/mcap.hpp"
#include "foxglove/server.hpp"
#include "simulator_utils.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>

using namespace foxglove::schemas;

class Simulator {
public:
  Simulator() = delete;
  explicit Simulator(const YAML::Node& config);
  ~Simulator();

  bool start();  // start the simulation thread
  void stop();   // stop the simulation thread

  StateInfo get_ego_state() const;
  void set_ego_control_input(const ControlInput& input);
  void update_planning_info(const planning::protos::PlanningInfo& info);
  std::shared_ptr<ReferenceLine> get_reference_line(void) const;
  std::shared_ptr<common::ObstacleList> get_obstacle_list(void) const;

private:
  void simulation_loop(void);
  void update_ego_state(void);
  bool register_publish_channels(void);
  SceneUpdate get_ego_scene_update(void) const;
  SceneUpdate get_trajectory_scene_update(void) const;
  SceneUpdate get_sampled_scene_update(void) const;
  SceneUpdate get_reference_line_scene_update(void) const;
  SceneUpdate get_obstacle_list_scene_update(size_t sim_world_step);
  SceneUpdate get_prediction_scene_update(size_t sim_world_step);
  SceneUpdate get_lanelets_scene_update(
      const std::vector<std::shared_ptr<Lanelet>>& lanelets) const;

private:
  double perception_range_m_ = 100.0;
  StateInfo ego_state_;
  VehicleInfo vehicle_info_;
  planning::protos::PlanningInfo planning_info_;
  std::shared_ptr<ReferenceLine> routing_reference_line_{nullptr};
  std::shared_ptr<ReferenceLine> reference_line_{nullptr};
  std::shared_ptr<spdlog::logger> logger_{nullptr};

  double last_ego_update_time_ = 0.0;

  std::thread sim_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> planning_info_updated_{false};
  mutable std::shared_mutex ego_state_mutex_;
  mutable std::shared_mutex planning_info_mutex_;
  mutable std::shared_mutex reference_line_mutex_;
  mutable std::shared_mutex obstacle_prediction_mutex_;

  std::unique_ptr<World> sim_world_{nullptr};

  bool save_mcap_{false};
  std::unique_ptr<foxglove::McapWriter> mcap_writer_{nullptr};
  std::unique_ptr<foxglove::WebSocketServer> socket_server_{nullptr};
  std::unique_ptr<foxglove::RawChannel> loop_runtime_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> ego_car_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> lanelet_scene_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> obstacle_list_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> obstacle_prediction_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> reference_line_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> trajectory_channel_{nullptr};
  std::unique_ptr<SceneUpdateChannel> sampled_channel_{nullptr};
  std::unique_ptr<FrameTransformChannel> transform_channel_{nullptr};
  std::unique_ptr<foxglove::RawChannel> planning_info_channel_{nullptr};

  std::unordered_set<std::string> obstacle_entity_ids_{};
  std::unordered_map<std::string, std::vector<PathPoint>> obstacle_predictions_{};
};
