/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:30:09
 * @LastEditTime: 2025-11-30 00:01:26
 * @FilePath: /mppi-in-autonomous-driving/planning/trajectory_cost.cu
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "trajectory_cost.cuh"

#include <cmath>

TrajectoryCost::TrajectoryCost(cudaStream_t stream) { bindToStream(stream); }

TrajectoryCost::~TrajectoryCost() { freeCudaMem(); }

void TrajectoryCost::setWaypoints(const std::shared_ptr<ReferenceLine>& waypoints) {
  host_waypoints_ = waypoints;

  // Reset cache when waypoints change
  host_last_matched_idx_ = 0;
  params_.reference_line.last_matched_idx = 0;

  // Free old device memory if exists
  if (device_waypoints_) {
    cudaFree(device_waypoints_);
    device_waypoints_ = nullptr;
  }
  if (device_left_road_edge_) {
    cudaFree(device_left_road_edge_);
    device_left_road_edge_ = nullptr;
  }
  if (device_right_road_edge_) {
    cudaFree(device_right_road_edge_);
    device_right_road_edge_ = nullptr;
  }

  if (waypoints && waypoints->size() > 0) {
    size_t num_waypoints = waypoints->size();

    // Convert ReferenceLine data to float3 array for GPU
    std::vector<float3> temp_waypoints(num_waypoints);
    const auto& x_vec = waypoints->get_x();
    const auto& y_vec = waypoints->get_y();
    const auto& yaw_vec = waypoints->get_yaw();

    for (size_t i = 0; i < num_waypoints; ++i) {
      temp_waypoints[i].x = static_cast<float>(x_vec[i]);
      temp_waypoints[i].y = static_cast<float>(y_vec[i]);
      temp_waypoints[i].z = static_cast<float>(yaw_vec[i]);  // Store yaw in z component
    }

    // Allocate device memory for waypoints
    size_t waypoints_size = num_waypoints * sizeof(float3);
    cudaMalloc((void**)&device_waypoints_, waypoints_size);

    // Copy waypoints to device
    cudaMemcpyAsync(device_waypoints_, temp_waypoints.data(), waypoints_size,
                    cudaMemcpyHostToDevice, stream_);

    // Convert and copy road edge distances
    const auto& left_edge = waypoints->get_left_road_edge();
    const auto& right_edge = waypoints->get_right_road_edge();

    if (!left_edge.empty() && left_edge.size() == num_waypoints) {
      std::vector<float> temp_left(num_waypoints);
      for (size_t i = 0; i < num_waypoints; ++i) {
        temp_left[i] = static_cast<float>(left_edge[i]);
      }

      size_t edge_size = num_waypoints * sizeof(float);
      cudaMalloc((void**)&device_left_road_edge_, edge_size);
      cudaMemcpyAsync(device_left_road_edge_, temp_left.data(), edge_size,
                      cudaMemcpyHostToDevice, stream_);
    }

    if (!right_edge.empty() && right_edge.size() == num_waypoints) {
      std::vector<float> temp_right(num_waypoints);
      for (size_t i = 0; i < num_waypoints; ++i) {
        temp_right[i] = static_cast<float>(right_edge[i]);
      }

      size_t edge_size = num_waypoints * sizeof(float);
      cudaMalloc((void**)&device_right_road_edge_, edge_size);
      cudaMemcpyAsync(device_right_road_edge_, temp_right.data(), edge_size,
                      cudaMemcpyHostToDevice, stream_);
    }

    // Update params with device pointers
    params_.reference_line.waypoints = device_waypoints_;
    params_.reference_line.left_road_edge_dist = device_left_road_edge_;
    params_.reference_line.right_road_edge_dist = device_right_road_edge_;
    params_.reference_line.count = static_cast<int>(num_waypoints);

    // Synchronize to ensure copy is complete
    cudaStreamSynchronize(stream_);
  } else {
    params_.reference_line.waypoints = nullptr;
    params_.reference_line.left_road_edge_dist = nullptr;
    params_.reference_line.right_road_edge_dist = nullptr;
    params_.reference_line.count = 0;
  }

  // Update params on device if GPU is already setup
  if (GPUMemStatus_) {
    paramsToDevice();
  }
}

void TrajectoryCost::setObstacles(const std::shared_ptr<common::ObstacleList>& obstacle_list) {
  host_obstacles_ = obstacle_list;

  // Free old device memory if exists
  if (device_obstacles_) {
    // Free trajectory arrays first
    for (auto& traj_ptr : device_trajectories_) {
      if (traj_ptr) {
        cudaFree(traj_ptr);
      }
    }
    device_trajectories_.clear();

    cudaFree(device_obstacles_);
    device_obstacles_ = nullptr;
  }

  if (obstacle_list && !obstacle_list->obstacles().empty()) {
    const auto& obstacles = obstacle_list->obstacles();
    size_t num_obstacles = obstacles.size();

    // Convert Obstacle data to ObstacleDevice struct array
    std::vector<ObstacleDevice> temp_obstacles(num_obstacles);
    device_trajectories_.resize(num_obstacles, nullptr);

    for (size_t i = 0; i < num_obstacles; ++i) {
      const auto& obs = obstacles[i];

      temp_obstacles[i].x = static_cast<float>(obs.x());
      temp_obstacles[i].y = static_cast<float>(obs.y());
      temp_obstacles[i].width = static_cast<float>(obs.width());
      temp_obstacles[i].length = static_cast<float>(obs.length());
      temp_obstacles[i].heading = static_cast<float>(obs.heading());
      temp_obstacles[i].type = static_cast<int>(obs.type());
      temp_obstacles[i].is_static = obs.is_static();

      // Handle future trajectory if available
      const auto& prediction = obs.prediction();
      if (!prediction.empty()) {
        // Allocate device memory for this obstacle's trajectory
        size_t traj_size = prediction.size() * sizeof(float3);
        cudaMalloc((void**)&device_trajectories_[i], traj_size);

        // Convert PathPoint to float3 [x, y, heading]
        std::vector<float3> traj_data(prediction.size());
        for (size_t j = 0; j < prediction.size(); ++j) {
          traj_data[j].x = static_cast<float>(prediction[j].x);
          traj_data[j].y = static_cast<float>(prediction[j].y);
          traj_data[j].z = static_cast<float>(prediction[j].yaw);
        }

        // Copy trajectory to device
        cudaMemcpyAsync(device_trajectories_[i], traj_data.data(), traj_size,
                        cudaMemcpyHostToDevice, stream_);

        temp_obstacles[i].future_traj = device_trajectories_[i];
        temp_obstacles[i].traj_len = static_cast<int>(prediction.size());
      } else {
        temp_obstacles[i].future_traj = nullptr;
        temp_obstacles[i].traj_len = 0;
      }
      temp_obstacles[i].reserved = 0;
    }

    // Allocate device memory for obstacle array
    size_t obstacles_size = num_obstacles * sizeof(ObstacleDevice);
    cudaMalloc((void**)&device_obstacles_, obstacles_size);

    // Copy obstacles to device
    cudaMemcpyAsync(device_obstacles_, temp_obstacles.data(), obstacles_size,
                    cudaMemcpyHostToDevice, stream_);

    // Update params with device pointer
    params_.obstacle_list.obstacles = device_obstacles_;
    params_.obstacle_list.count = static_cast<int>(num_obstacles);

    // Synchronize to ensure copy is complete
    cudaStreamSynchronize(stream_);
  } else {
    params_.obstacle_list.obstacles = nullptr;
    params_.obstacle_list.count = 0;
  }

  // Update params on device if GPU is already setup
  if (GPUMemStatus_) {
    paramsToDevice();
  }
}

void TrajectoryCost::updateMatchedIndex(float x, float y) {
  if (!host_waypoints_ || host_waypoints_->size() == 0) {
    return;
  }

  // Find the nearest point and update the cache
  int nearest_idx = 0;
  computeDistanceToReferenceLine(x, y, &nearest_idx, nullptr);

  // Cache is already updated in computeDistanceToReferenceLine
  // Sync to device params
  params_.reference_line.last_matched_idx = host_last_matched_idx_;

  if (GPUMemStatus_) {
    paramsToDevice();
  }
}

void TrajectoryCost::paramsToDevice() {
  if (GPUMemStatus_) {
    // Copy the params struct (which now contains device pointer and count)
    cudaMemcpyAsync(&cost_d_->params_, &params_, sizeof(TrajectoryCostParams),
                    cudaMemcpyHostToDevice, stream_);
    cudaStreamSynchronize(stream_);
  }
}

void TrajectoryCost::freeCudaMem() {
  // Free waypoints device memory
  if (device_waypoints_) {
    cudaFree(device_waypoints_);
    device_waypoints_ = nullptr;
  }

  // Free road edge distances device memory
  if (device_left_road_edge_) {
    cudaFree(device_left_road_edge_);
    device_left_road_edge_ = nullptr;
  }
  if (device_right_road_edge_) {
    cudaFree(device_right_road_edge_);
    device_right_road_edge_ = nullptr;
  }

  // Free obstacles device memory
  if (device_obstacles_) {
    // Free trajectory arrays first
    for (auto& traj_ptr : device_trajectories_) {
      if (traj_ptr) {
        cudaFree(traj_ptr);
      }
    }
    device_trajectories_.clear();

    cudaFree(device_obstacles_);
    device_obstacles_ = nullptr;
  }

  // Call base class cleanup
  Cost<TrajectoryCost, TrajectoryCostParams, VehicleDynamicsParams>::freeCudaMem();
}

float TrajectoryCost::computeDistanceToReferenceLine(float x, float y, int* nearest_idx,
                                                     float* matched_heading) const {
  if (!host_waypoints_ || host_waypoints_->size() == 0) {
    return 0.0f;
  }

  const auto& x_vec = host_waypoints_->get_x();
  const auto& y_vec = host_waypoints_->get_y();
  const size_t n = x_vec.size();

  float min_dist = std::numeric_limits<float>::max();
  int min_idx = host_last_matched_idx_;

  // Local search window (same as device version)
  const int local_window = 10;
  int search_start = std::max(0, host_last_matched_idx_ - 2);
  int search_end = std::min(static_cast<int>(n) - 1, host_last_matched_idx_ + local_window);
  if (host_last_matched_idx_ == 0) {
    // If no cache, search the whole range
    search_start = 0;
    search_end = static_cast<int>(n) - 1;
  }

  // Local search: only check nearby points for temporal continuity
  for (int i = search_start; i <= search_end; ++i) {
    float dx = x - static_cast<float>(x_vec[i]);
    float dy = y - static_cast<float>(y_vec[i]);
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  // Update cache for next call
  host_last_matched_idx_ = min_idx;

  // Fine search: check interpolation between nearest point and neighbors
  if (min_idx > 0 && min_idx < static_cast<int>(n) - 1) {
    // Check segment [min_idx-1, min_idx]
    float x0 = static_cast<float>(x_vec[min_idx - 1]);
    float y0 = static_cast<float>(y_vec[min_idx - 1]);
    float x1 = static_cast<float>(x_vec[min_idx]);
    float y1 = static_cast<float>(y_vec[min_idx]);

    float dx_seg = x1 - x0;
    float dy_seg = y1 - y0;
    float seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg;

    if (seg_len_sq > 1e-6f) {
      float t = ((x - x0) * dx_seg + (y - y0) * dy_seg) / seg_len_sq;
      t = std::max(0.0f, std::min(1.0f, t));  // Clamp to [0, 1]

      float proj_x = x0 + t * dx_seg;
      float proj_y = y0 + t * dy_seg;
      float dist = std::sqrt((x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y));
      min_dist = std::min(min_dist, dist);
    }

    // Check segment [min_idx, min_idx+1]
    x0 = x1;
    y0 = y1;
    x1 = static_cast<float>(x_vec[min_idx + 1]);
    y1 = static_cast<float>(y_vec[min_idx + 1]);

    dx_seg = x1 - x0;
    dy_seg = y1 - y0;
    seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg;

    if (seg_len_sq > 1e-6f) {
      float t = ((x - x0) * dx_seg + (y - y0) * dy_seg) / seg_len_sq;
      t = std::max(0.0f, std::min(1.0f, t));

      float proj_x = x0 + t * dx_seg;
      float proj_y = y0 + t * dy_seg;
      float dist = std::sqrt((x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y));
      min_dist = std::min(min_dist, dist);
    }
  }

  if (nearest_idx) {
    *nearest_idx = min_idx;
  }

  double rx = x_vec[min_idx];
  double ry = y_vec[min_idx];
  double heading = host_waypoints_->get_yaw()[min_idx];
  float latral_sign =
      ((x - rx) * (-std::sin(heading)) + (y - ry) * std::cos(heading)) >= 0 ? 1.0f : -1.0f;
  min_dist *= latral_sign;

  if (matched_heading) {
    *matched_heading = static_cast<float>(heading);
  }

  return min_dist;
}

float TrajectoryCost::computeCostInternal(const Eigen::Ref<const output_array> s) const {
  float lateral_distance = 0.0f;
  float matched_heading = 0.0f;
  const float x = s(S_IDX(POS_X));
  const float y = s(S_IDX(POS_Y));
  const float velocity = s(S_IDX(VELOCITY));
  const float heading = s(S_IDX(YAW));
  const float accel = s(S_IDX(ACCEL));
  const float steer = s(S_IDX(STEER));
  int matched_idx = -1;

  // Compute distance to reference line if waypoints are available
  if (host_waypoints_ && host_waypoints_->size() > 0) {
    lateral_distance = computeDistanceToReferenceLine(x, y, &matched_idx, &matched_heading);
  }

  float violation = 0.0f;
  if (accel >= params_.max_accel || accel <= params_.min_accel ||
      steer >= params_.max_steer_angle || steer <= params_.min_steer_angle || velocity < -0.0001f) {
    violation = 500.0f;
  }

  if (matched_idx != -1) {
    if (lateral_distance > 0 &&
        host_waypoints_->get_left_road_edge()[matched_idx] < std::abs(lateral_distance)) {
      violation = 500.0f;
    } else if (lateral_distance < 0 &&
               host_waypoints_->get_right_road_edge()[matched_idx] < std::abs(lateral_distance)) {
      violation = 500.0f;
    }
  }

  // Compute heading error (normalize to [-pi, pi])
  float heading_error = heading - matched_heading;
  if (heading_error > M_PI) {
    heading_error -= 2.0f * M_PI;
  } else if (heading_error < -M_PI) {
    heading_error += 2.0f * M_PI;
  }

  float obstacle_cost = computeObstacleCost(x, y, heading);

  float position_cost = params_.position_coeff * std::abs(lateral_distance);
  float velocity_cost = params_.velocity_coeff * std::abs(velocity - params_.target_velocity);
  float angle_cost = params_.angle_coeff * std::abs(heading_error);
  float accel_cost = params_.accel_effort_coeff * std::abs(accel);
  float steer_cost = params_.steer_effort_coeff * std::abs(steer);
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost + obstacle_cost +
         violation;
}

float TrajectoryCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep,
                                       int* crash_status) {
  return computeCostInternal(s);
}

float TrajectoryCost::terminalCost(const Eigen::Ref<const output_array> s) {
  return computeCostInternal(s);
}

float TrajectoryCost::computeControlCost(const Eigen::Ref<const control_array> u, int timestep,
                                         int* crash) {
  return params_.control_cost_coeff[0] * std::abs(u[0]) +
         params_.control_cost_coeff[1] * std::abs(u[1]);
}

float TrajectoryCost::computeObstacleCost(float x, float y, float heading) const {
  if (!host_obstacles_ || host_obstacles_->obstacles().empty()) {
    return 0.0f;
  }

  float total_cost = 0.0f;
  const auto& obstacles = host_obstacles_->obstacles();
  float3 ego_pose{x, y, heading};

  VehicleCorners vehicle_corners = computeVehicleCorners(ego_pose);

  for (const auto& obs : obstacles) {
    float3 obstacle_pose{static_cast<float>(obs.x()), static_cast<float>(obs.y()),
                         static_cast<float>(obs.heading())};
    float half_length =
        static_cast<float>(obs.length()) / 2.0f + params_.longitudinal_safety_margin;
    float half_width = static_cast<float>(obs.width()) / 2.0f + params_.lateral_safety_margin;

    // Use shared host/device function
    bool in_capsule = cornersInCapsule(vehicle_corners, obstacle_pose, half_length, half_width);
    if (in_capsule) {
      total_cost += 1000.0f;
    }

    // Future trajectory cost (if available)
    const auto& prediction = obs.prediction();
    if (!prediction.empty()) {
      // TODO: Add temporal collision checking with predicted trajectory
      // This allows checking collision at future timesteps along the prediction horizon
    }
  }

  return total_cost;
}

float TrajectoryCost::computeSafetyMargin(const Eigen::Vector2f& ego_position,
                                          const Eigen::Vector3f& obstacle_pose,
                                          const Eigen::Vector2f& ellipse_axes) const {
  const float diff_x = obstacle_pose.x() - ego_position.x();
  const float diff_y = obstacle_pose.y() - ego_position.y();
  const float cos_heading = std::cos(obstacle_pose.z());
  const float sin_heading = std::sin(obstacle_pose.z());

  float rotated_x = diff_x * cos_heading + diff_y * sin_heading;
  float rotated_y = -diff_x * sin_heading + diff_y * cos_heading;
  float norm = (rotated_x * rotated_x) / (ellipse_axes.x() * ellipse_axes.x()) +
               (rotated_y * rotated_y) / (ellipse_axes.y() * ellipse_axes.y());

  return norm - 1.0f;
}
