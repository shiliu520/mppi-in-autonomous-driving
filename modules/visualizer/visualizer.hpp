/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2026-01-19 00:00:00
 * @LastEditTime: 2026-01-20 00:06:08
 * @FilePath: /mppi-in-autonomous-driving/simulator/visualizer.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "common/obstacle.hpp"
#include "common/protos/planning_info.pb.h"
#include "common/reference_line.hpp"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "foxglove/foxglove.hpp"
#include "foxglove/mcap.hpp"
#include "foxglove/server.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

class Visualizer {
public:
  Visualizer() = delete;
  explicit Visualizer(const YAML::Node& config);
  ~Visualizer();

  // Initialization and lifecycle management
  bool start();
  void stop();

  // Set world reference for visualization
  void set_road_network(const std::shared_ptr<RoadNetwork>& road_network) {
    road_network_ = road_network;
  }

  // Publish visualization data
  void log_ego_state(const StateInfo& ego_state);
  void log_transform(const foxglove::schemas::FrameTransform& transform);
  void log_loop_runtime(double elapsed_seconds);
  void log_planning_info(const planning::protos::PlanningInfo& planning_info);
  void log_trajectory(const planning::protos::PlanningInfo& planning_info);
  void log_sampled_trajectories(const planning::protos::PlanningInfo& planning_info);
  void log_reference_line(const std::shared_ptr<ReferenceLine>& reference_line);
  void log_lanelets(const std::vector<std::shared_ptr<Lanelet>>& lanelets);
  void log_obstacles(const std::vector<std::shared_ptr<Obstacle>>& obstacles, size_t sim_world_step,
                     const StateInfo& current_ego_state, double perception_range_m);
  void log_obstacle_predictions(
      const std::vector<std::shared_ptr<Obstacle>>& obstacles,
      const std::unordered_map<std::string, std::vector<PathPoint>>& predict_trajs,
      size_t sim_world_step, const StateInfo& current_ego_state, double perception_range_m);

private:
  // Channel registration and initialization
  bool register_publish_channels();

  // Scene update generation methods (extracted from Simulator)
  foxglove::schemas::SceneUpdate get_ego_scene_update(const StateInfo& ego_state) const;
  foxglove::schemas::SceneUpdate get_trajectory_scene_update(
      const planning::protos::PlanningInfo& planning_info) const;
  foxglove::schemas::SceneUpdate get_sampled_scene_update(
      const planning::protos::PlanningInfo& planning_info) const;
  foxglove::schemas::SceneUpdate get_reference_line_scene_update(
      const std::shared_ptr<ReferenceLine>& reference_line);
  foxglove::schemas::SceneUpdate get_obstacle_list_scene_update(
      const std::vector<std::shared_ptr<Obstacle>>& obstacles, size_t sim_world_step,
      const StateInfo& current_ego_state, double perception_range_m);
  foxglove::schemas::SceneUpdate get_prediction_scene_update(
      const std::vector<std::shared_ptr<Obstacle>>& obstacles,
      const std::unordered_map<std::string, std::vector<PathPoint>>& predict_trajs,
      size_t sim_world_step, const StateInfo& current_ego_state, double perception_range_m);
  foxglove::schemas::SceneUpdate get_lanelets_scene_update(
      const std::vector<std::shared_ptr<Lanelet>>& lanelets);

private:
  // Logger
  std::shared_ptr<spdlog::logger> logger_{nullptr};

  // Configuration
  bool save_mcap_{false};

  // World reference
  std::shared_ptr<RoadNetwork> road_network_{nullptr};

  std::unique_ptr<foxglove::McapWriter> mcap_writer_{nullptr};
  std::unique_ptr<foxglove::WebSocketServer> socket_server_{nullptr};

  // Channels for different data types
  std::unique_ptr<foxglove::RawChannel> loop_runtime_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> ego_car_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> lanelet_scene_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> obstacle_list_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> obstacle_prediction_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> reference_line_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> trajectory_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> sampled_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::FrameTransformChannel> transform_channel_{nullptr};
  std::unique_ptr<foxglove::RawChannel> planning_info_channel_{nullptr};

  // State tracking
  std::atomic<bool> running_{false};
  std::atomic<bool> reference_line_initialized_{false};
  std::atomic<bool> lanelets_initialized_{false};
};
