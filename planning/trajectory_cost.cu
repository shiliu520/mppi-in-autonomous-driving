/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:30:09
 * @LastEditTime: 2025-11-24 23:50:29
 * @FilePath: /mppi-in-autonomous-driving/planning/trajectory_cost.cu
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "trajectory_cost.cuh"

#include <cmath>

TrajectoryCost::TrajectoryCost(cudaStream_t stream) { 
  bindToStream(stream); 
}

TrajectoryCost::~TrajectoryCost() {
  freeCudaMem();
}

void TrajectoryCost::setWaypoints(const std::shared_ptr<ReferenceLine>& waypoints) {
  host_waypoints_ = waypoints;
  
  // Reset cache when waypoints change
  host_last_matched_idx_ = 0;
  params_.last_matched_idx = 0;
  
  // Free old device memory if exists
  if (device_waypoints_) {
    cudaFree(device_waypoints_);
    device_waypoints_ = nullptr;
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
    
    // Update params with device pointer
    params_.waypoints = device_waypoints_;
    params_.waypoints_count = static_cast<int>(num_waypoints);
    
    // Synchronize to ensure copy is complete
    cudaStreamSynchronize(stream_);
  } else {
    params_.waypoints = nullptr;
    params_.waypoints_count = 0;
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
  computeDistanceToReferenceLine(x, y, &nearest_idx);
  
  // Cache is already updated in computeDistanceToReferenceLine
  // Sync to device params
  params_.last_matched_idx = host_last_matched_idx_;
  
  if (GPUMemStatus_) {
    paramsToDevice();
  }
}

void TrajectoryCost::paramsToDevice() {
  if (GPUMemStatus_) {
    // Copy the params struct (which now contains device pointer and count)
    cudaMemcpyAsync(&cost_d_->params_, &params_, 
                    sizeof(TrajectoryCostParams), 
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
  
  // Call base class cleanup
  Cost<TrajectoryCost, TrajectoryCostParams, VehicleDynamicsParams>::freeCudaMem();
}

float TrajectoryCost::computeDistanceToReferenceLine(float x, float y, int* nearest_idx) const {
  if (!host_waypoints_ || host_waypoints_->size() == 0) {
    return 0.0f;
  }
  
  const auto& x_vec = host_waypoints_->get_x();
  const auto& y_vec = host_waypoints_->get_y();
  const size_t n = x_vec.size();
  
  float min_dist = std::numeric_limits<float>::max();
  int min_idx = host_last_matched_idx_;
  
  // Local search window (same as device version)
  const int local_window = 30;
  int search_start = std::max(0, host_last_matched_idx_ - 5);
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
  
  return min_dist;
}

float TrajectoryCost::computeCostInternal(float x, float y, float velocity, float heading,
                                          float accel, float steer) const {
  float lateral_distance = 0.0f;
  
  // Compute distance to reference line if waypoints are available
  if (host_waypoints_ && host_waypoints_->size() > 0) {
    lateral_distance = computeDistanceToReferenceLine(x, y);
  }

  float violation = 0.0f;
  if (accel >= params_.max_accel || accel <= params_.min_accel || 
      steer >= params_.max_steer_angel || steer <= params_.min_steer_angel) {
    violation = 500;
  }

  float position_cost = params_.bike_position_coeff * lateral_distance;
  float velocity_cost = params_.bike_velocity_coeff * std::abs(velocity - params_.target_velocity);
  float angle_cost = params_.bike_angle_coeff * std::abs(heading);
  float accel_cost = params_.accel_effort_coeff * std::abs(accel);
  float steer_cost = params_.steer_effort_coeff * std::abs(steer);
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost + violation;
}

float TrajectoryCost::computeStateCost(const Eigen::Ref<const output_array> s, int timestep,
                                       int* crash_status) {
  return computeCostInternal(s(0), s(1), s(2), s(3), s(4), s(5));
}

float TrajectoryCost::terminalCost(const Eigen::Ref<const output_array> s) {
  return computeCostInternal(s(0), s(1), s(2), s(3), s(4), s(5));
}

float TrajectoryCost::computeControlCost(const Eigen::Ref<const control_array> u, int timestep,
                                         int* crash) {
  return params_.control_cost_coeff[0] * std::abs(u[0]) +
         params_.control_cost_coeff[1] * std::abs(u[1]);
}
