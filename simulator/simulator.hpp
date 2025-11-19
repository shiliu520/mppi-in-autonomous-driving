/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:57:28
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-11-18 23:51:39
 * @FilePath: /mppi-in-autonomous-driving/simulator/simulator.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "foxglove/foxglove.hpp"
#include "foxglove/mcap.hpp"
#include "foxglove/server.hpp"
#include "simulator_utils.hpp"

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

class Simulator {
public:
  Simulator();
  ~Simulator();

  bool start();  // start the simulation thread
  void stop();   // stop the simulation thread

  StateInfo get_ego_state() const;
  void set_ego_control_input(const ControlInput& input);

private:
  void simulation_loop(void);
  void update_ego_state(void);
  bool register_publish_channels(void);
  foxglove::schemas::SceneUpdate get_ego_scene_update(const foxglove::schemas::Pose& ego_pose);
  foxglove::schemas::SceneUpdate get_lane_scene_update(const foxglove::schemas::Pose& ego_pose);
  foxglove::schemas::SceneUpdate get_trajectory_scene_update(void) const;

private:
  StateInfo ego_state_;
  VehicleInfo vehicle_info_;
  std::shared_ptr<spdlog::logger> logger_ = nullptr;

  double last_ego_update_time_ = 0.0;

  std::thread sim_thread_;
  std::atomic<bool> running_{false};
  mutable std::shared_mutex ego_state_mutex_;

  bool save_mcap_{false};
  std::unique_ptr<foxglove::McapWriter> mcap_writer_{nullptr};
  std::unique_ptr<foxglove::WebSocketServer> socket_server_{nullptr};
  std::unique_ptr<foxglove::RawChannel> loop_runtime_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> ego_car_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> lane_lines_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::SceneUpdateChannel> trajectory_channel_{nullptr};
  std::unique_ptr<foxglove::schemas::FrameTransformChannel> transform_channel_{nullptr};
  std::unique_ptr<foxglove::RawChannel> planning_info_channel_{nullptr};
};
