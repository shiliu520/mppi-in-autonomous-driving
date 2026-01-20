/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:57:28
 * @LastEditTime: 2026-01-21 00:22:14
 * @FilePath: /mppi-in-autonomous-driving/modules/simulator/simulator.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "simulator.hpp"
#include "simulator_utils.hpp"
#include "common/cubic_spline.hpp"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "simple_predictor.hpp"

#include <algorithm>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>
#include <vector>

Simulator::Simulator(const YAML::Node& config, std::shared_ptr<Visualizer> visualizer)
    : config_(config), visualizer_(visualizer) {
  auto simulator_config = config["simulator"];
  std::string simulator_log_level = simulator_config["log_level"].as<std::string>("info");
  logger_ = create_logger("simulator_logger", simulator_log_level);

  std::string scenario_file_path = simulator_config["scenario_file_path"].as<std::string>("");
  if (!scenario_file_path.empty() && scenario_file_path[0] != '/') {
    scenario_file_path = std::string(PROJECT_SOURCE_DIR) + "/" + scenario_file_path;
  }
  scenario_file_path_ = scenario_file_path;

  perception_range_m_ = simulator_config["perception_range_m"].as<double>(100.0);

  vehicle_info_.wheel_base = config["vehicle_info"]["wheel_base_m"].as<double>(2.8);
  int planning_problem_id = simulator_config["planning_problem_id"].as<int>(1);

  const auto& [obstacles_scenario, road_network_scenario, time_step_size, planning_problems] =
      InputUtils::getDataFromCommonRoad(scenario_file_path);
  assert(!planning_problems.empty() && "No planning problems found in scenario file");

  std::shared_ptr<PlanningProblem> selected_planning_problem = nullptr;
  for (const auto& problem : planning_problems) {
    if (problem->getId() == planning_problem_id) {
      selected_planning_problem = problem;
      break;
    }
  }

  if (selected_planning_problem == nullptr) {
    selected_planning_problem = planning_problems[0];
    LOG_WARN(logger_,
             "Requested planning_problem_id {} not found; using planning_problem_id {} instead.",
             planning_problem_id, selected_planning_problem->getId());
  }
  planning_problem_id_ = static_cast<int>(selected_planning_problem->getId());

  const auto& initial_state = selected_planning_problem->getInitialState();
  ego_state_.x = initial_state->getXPosition();
  ego_state_.y = initial_state->getYPosition();
  ego_state_.velocity = initial_state->getVelocity();
  ego_state_.heading = initial_state->getOrientation();
  ego_state_.accel = initial_state->getAcceleration();
  ego_state_.steer = 0;

  WorldParameters wp{RoadNetworkParameters(), SensorParameters(), ActuatorParameters::egoDefaults(),
                     TimeParameters(5, 0.3, 0.1), ActuatorParameters::vehicleDefaults()};
  sim_world_ = std::make_unique<World>("mppi_simulation", 0, road_network_scenario,
                                       std::vector<std::shared_ptr<Obstacle>>{}, obstacles_scenario,
                                       time_step_size, wp);

  LOG_INFO(logger_, "Loaded scenario from {}, using planning problem ID: {}", scenario_file_path,
           selected_planning_problem->getId());

  const std::shared_ptr<LaneletGraph>& topo_graph =
      sim_world_->getRoadNetwork()->getTopologicalMap();
  size_t start_lanelet_id = 0;
  size_t goal_lanelet_id = 0;
  const auto start_lanelets_vector =
      sim_world_->getRoadNetwork()->findLaneletsByPosition(ego_state_.x, ego_state_.y);
  if (!start_lanelets_vector.empty()) {
    start_lanelet_id = start_lanelets_vector[0]->getId();
    LOG_INFO(logger_, "Ego vehicle start lanelet ID: {}", start_lanelet_id);
  } else {
    LOG_ERROR(logger_, "Ego vehicle start position not in any lanelet!");
  }
  const auto goal_states_vector = selected_planning_problem->getGoalStates();
  if (!goal_states_vector.empty()) {
    const auto goal_positions = goal_states_vector[0].getGoalPositions();
    if (!goal_positions.empty()) {
      const uint32_t* goal_position = std::get_if<uint32_t>(&goal_positions[0]);
      if (goal_position != nullptr) {
        goal_lanelet_id = *goal_position;
        LOG_INFO(logger_, "Ego vehicle goal lanelet ID: {}", goal_lanelet_id);
      } else {
        LOG_ERROR(logger_, "Goal position is not specified as a lanelet ID!");
      }
    } else {
      LOG_ERROR(logger_, "No goal positions specified in goal state!");
    }
  } else {
    LOG_ERROR(logger_, "No goal states specified in planning problem!");
  }

  auto paths = topo_graph->findPaths(start_lanelet_id, goal_lanelet_id, true);
  if (!paths.empty()) {
    std::vector<double> ref_xs;
    std::vector<double> ref_ys;
    bool is_lane_change = false;
    for (size_t idx = 0; idx < paths.size(); ++idx) {
      auto lanelet = sim_world_->getRoadNetwork()->findLaneletById(paths[idx]);
      if (idx + 1 < paths.size()) {
        LOG_INFO(logger_, "Routing lanelet {} -> {}", paths[idx], paths[idx + 1]);
        auto next_lanelet = sim_world_->getRoadNetwork()->findLaneletById(paths[idx + 1]);
        if (lanelet_operations::areLaneletsAdjacent(lanelet, next_lanelet)) {
          is_lane_change = true;
          continue;
        }
      }
      for (const auto& vertex : lanelet->getCenterVertices()) {
        double interval_distance = 100.0;
        if (ref_xs.size() > 0) {
          const double last_x = ref_xs.back();
          const double last_y = ref_ys.back();
          interval_distance =
              std::sqrt(std::pow(vertex.x - last_x, 2) + std::pow(vertex.y - last_y, 2));
          if (interval_distance < 0.1) {
            continue;
          }
        }
        if (is_lane_change) {
          if (interval_distance > 12) {
            is_lane_change = false;
          }
          continue;
        }
        ref_xs.push_back(vertex.x);
        ref_ys.push_back(vertex.y);
      }
    }

    routing_reference_line_ = std::make_shared<ReferenceLine>(ref_xs, ref_ys, 0.5);
    LOG_INFO(logger_, "Reference line created with {} points, length {:.2f} m",
             routing_reference_line_->size(), routing_reference_line_->length());

    // Compute distance to left and right road edges for each reference line point
    std::vector<double> left_road_edge_distances;
    std::vector<double> right_road_edge_distances;
    left_road_edge_distances.reserve(routing_reference_line_->size());
    right_road_edge_distances.reserve(routing_reference_line_->size());

    for (size_t i = 0; i < routing_reference_line_->size(); ++i) {
      // Compute distances to left and right road edges
      const auto point = routing_reference_line_->at(i);
      auto [left_dist, right_dist] =
          compute_road_edge_distances(point.x(), point.y(), sim_world_->getRoadNetwork());
      left_road_edge_distances.push_back(left_dist);
      right_road_edge_distances.push_back(right_dist);
    }

    routing_reference_line_->set_left_road_edge(left_road_edge_distances);
    routing_reference_line_->set_right_road_edge(right_road_edge_distances);
    LOG_INFO(logger_, "Computed road edge distances for reference line");
  } else {
    LOG_ERROR(logger_, "No path found from lanelet {} to {}", start_lanelet_id, goal_lanelet_id);
    routing_reference_line_ = nullptr;
  }
}

Simulator::~Simulator() { stop(); }

bool Simulator::start() {
  if (running_) {
    return true;
  }

  if (visualizer_) {
    visualizer_->set_road_network(sim_world_->getRoadNetwork());
  }

  running_ = true;
  sim_thread_ = std::thread(&Simulator::simulation_loop, this);
  LOG_INFO(logger_, "Simulator started.");

  return true;
}

void Simulator::stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }
  if (visualizer_) {
    visualizer_->stop();
  }
  LOG_INFO(logger_, "Simulator stopped.");
}

void Simulator::simulation_loop() {
  if (routing_reference_line_ && visualizer_) {
    visualizer_->log_reference_line(routing_reference_line_);
  }

  const auto loop_start = std::chrono::steady_clock::now();
  auto next_tick = loop_start;
  uint32_t loop_count = 0;
  while (running_) {
    update_ego_state();
    const auto ego_state = get_ego_state();

    auto dur = std::chrono::steady_clock::now() - loop_start;
    const double elapsed_seconds = std::chrono::duration<double>(dur).count();

    if (visualizer_) {
      visualizer_->log_loop_runtime(elapsed_seconds);
      visualizer_->log_ego_state(ego_state);
    }

    if (loop_count % 250 == 0) {
      // publish lanelets scene at 0.2 Hz
      if (visualizer_) {
        visualizer_->log_lanelets(sim_world_->getRoadNetwork()->getLaneletNetwork());
      }
    }
    if (loop_count % 5 == 0) {
      // publish obstacles scene at 10 Hz
      if (visualizer_) {
        visualizer_->log_obstacles(sim_world_->getObstacles(), sim_world_timestep_.load(),
                                   ego_state, perception_range_m_);
        {
          std::shared_lock<std::shared_mutex> lock(obstacle_prediction_mutex_);
          visualizer_->log_obstacle_predictions(sim_world_->getObstacles(), obstacle_predictions_,
                                                sim_world_timestep_.load(), ego_state,
                                                perception_range_m_);
        }
      }
      if (loop_count != 0) {
        sim_world_->propagate();
        ++sim_world_timestep_;
      }
    }

    ++loop_count;
    next_tick += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(next_tick);
  }
}

StateInfo Simulator::get_ego_state() const {
  std::shared_lock lock(ego_state_mutex_);
  return ego_state_;
}

void Simulator::set_ego_control_input(const ControlInput& input) {
  std::unique_lock lock(ego_state_mutex_);
  ego_state_.accel = input.accel;
  ego_state_.steer = input.steer;
}

void Simulator::update_ego_state() {
  const double current_time_sec = TimeUtil::NowSeconds();
  if (last_ego_update_time_ > 0.1) {
    const double dt = current_time_sec - last_ego_update_time_;
    std::unique_lock lock(ego_state_mutex_);
    ego_state_.x += ego_state_.velocity * std::cos(ego_state_.heading) * dt;
    ego_state_.y += ego_state_.velocity * std::sin(ego_state_.heading) * dt;
    ego_state_.heading +=
        ego_state_.velocity * (std::tan(ego_state_.steer) / vehicle_info_.wheel_base) * dt;
    ego_state_.velocity += ego_state_.accel * dt;
    if (ego_state_.velocity < 1e-6) {
      ego_state_.velocity = 0.0;
      if (ego_state_.accel < 0) {
        ego_state_.accel = 0.0;
      }
    }

    LOG_DEBUG(logger_,
              "update_ego_state >>> x: {:.2f}, y: {:.2f}, v: {:.2f}, heading: {:.5f} accel: {:.2f} "
              "steer: {:.5f}",
              ego_state_.x, ego_state_.y, ego_state_.velocity, ego_state_.heading, ego_state_.accel,
              ego_state_.steer);
  }
  last_ego_update_time_ = current_time_sec;
}

std::shared_ptr<ReferenceLine> Simulator::get_reference_line(void) const {
  std::shared_lock lock(reference_line_mutex_);
  if (routing_reference_line_) {
    return routing_reference_line_;
  }

  return nullptr;
}

std::shared_ptr<common::ObstacleList> Simulator::get_obstacle_list(void) {
  auto obstacle_list = std::make_shared<common::ObstacleList>();
  auto obstacles = sim_world_->getObstacles();
  StateInfo current_ego_state = get_ego_state();
  std::shared_lock<std::shared_mutex> lock(obstacle_prediction_mutex_);

  obstacle_predictions_.clear();
  for (const auto& obstacle : obstacles) {
    const double obstacle_x = obstacle->getCurrentState()->getXPosition();
    const double obstacle_y = obstacle->getCurrentState()->getYPosition();
    double distance_to_ego =
        std::hypot(obstacle_x - current_ego_state.x, obstacle_y - current_ego_state.y);
    size_t current_timestep = sim_world_timestep_.load(std::memory_order_relaxed);
    if (distance_to_ego > perception_range_m_ ||
        (!obstacle->isStatic() && (current_timestep > obstacle->getFinalTimeStep() ||
                                   current_timestep < obstacle->getFirstTimeStep()))) {
      continue;
    }

    common::Obstacle obs;
    std::string obs_id = std::to_string(obstacle->getId());
    obs.set_id(obs_id);
    obs.set_type(static_cast<common::ObstacleType>(obstacle->getObstacleType()));
    obs.set_x(obstacle_x);
    obs.set_y(obstacle_y);
    obs.set_length(obstacle->getGeoShape().getLength());
    obs.set_width(obstacle->getGeoShape().getWidth());
    obs.set_heading(obstacle->getCurrentState()->getGlobalOrientation());
    if (!obstacle->isStatic()) {
      obs.set_is_static(false);
    } else {
      obs.set_is_static(true);
    }

    auto predicted_trajectory = get_simple_prediction(obstacle, kHorizonLength);
    obs.set_prediction(predicted_trajectory);
    obstacle_predictions_[std::to_string(obstacle->getId())] = predicted_trajectory;

    obstacle_list->append(obs);
  }
  return obstacle_list;
}
