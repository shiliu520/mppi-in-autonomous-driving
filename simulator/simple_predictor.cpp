/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-12-09 23:01:36
 * @LastEditTime: 2025-12-10 23:37:54
 * @FilePath: /mppi-in-autonomous-driving/simulator/simple_predictor.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "simple_predictor.hpp"

std::vector<PathPoint> get_simple_prediction(const std::shared_ptr<Obstacle>& obstacle_ptr, size_t horizon_length) {
  std::vector<PathPoint> predicted_trajectory;

  auto obstacle_state_map = obstacle_ptr->getTrajectoryPrediction();

  // Convert state_map to sorted vector of (time_step, state) pairs
  std::vector<std::pair<size_t, std::shared_ptr<State>>> sorted_states;
  for (const auto& [time_step, state] : obstacle_state_map) {
    sorted_states.emplace_back(time_step, state);
  }
  std::sort(sorted_states.begin(), sorted_states.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  if (sorted_states.empty()) {
    return predicted_trajectory;
  }

  // If we have more states than horizon_length, truncate
  size_t num_states = std::min(sorted_states.size(), horizon_length);

  PathPoint cur_point;
  cur_point.x = obstacle_ptr->getCurrentState()->getXPosition();
  cur_point.y = obstacle_ptr->getCurrentState()->getYPosition();
  cur_point.yaw = obstacle_ptr->getCurrentState()->getGlobalOrientation();
  cur_point.v = obstacle_ptr->getCurrentState()->getVelocity();
  cur_point.t = 0.0;
  predicted_trajectory.emplace_back(cur_point);

  for (size_t i = 1; i < num_states; ++i) {
    const auto& state = sorted_states[i - 1].second;
    PathPoint point;
    point.x = state->getXPosition();
    point.y = state->getYPosition();
    point.yaw = state->getGlobalOrientation();
    point.v = state->getVelocity();
    point.t = i * kTimeStepSec;
    predicted_trajectory.emplace_back(point);
  }

  // If we have fewer states than horizon_length, extrapolate with constant velocity
  if (sorted_states.size() < horizon_length) {
    const auto& last_state = sorted_states.back().second;
    double last_x = last_state->getXPosition();
    double last_y = last_state->getYPosition();
    double last_yaw = last_state->getGlobalOrientation();
    double last_v = last_state->getVelocity();
    size_t last_time_step = sorted_states.back().first;

    for (size_t i = sorted_states.size(); i < horizon_length; ++i) {
      size_t time_step = last_time_step + (i - sorted_states.size() + 1);
      double dt = kTimeStepSec;

      last_x += last_v * std::cos(last_yaw) * dt;
      last_y += last_v * std::sin(last_yaw) * dt;

      PathPoint point;
      point.x = last_x;
      point.y = last_y;
      point.yaw = last_yaw;
      point.v = last_v;
      point.t = time_step * kTimeStepSec;
      predicted_trajectory.emplace_back(point);
    }
  }

  return predicted_trajectory;
}
