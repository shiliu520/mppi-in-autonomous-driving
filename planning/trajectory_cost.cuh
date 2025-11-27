/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:30:11
 * @LastEditTime: 2025-11-28 00:50:25
 * @FilePath: /mppi-in-autonomous-driving/planning/trajectory_cost.cuh
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/obstacle.hpp"
#include "common/reference_line.hpp"
#include "mppi/cost_functions/cost.cuh"
#include "mppi/utils/file_utils.h"
#include "vehicle_dynamics.cuh"

#include <cmath>

#ifndef M_PI_F
#define M_PI_F 3.14159265f
#endif
#ifndef M_2PI_F
#define M_2PI_F 6.28318530f
#endif
#ifndef SQ
#define SQ(x) ((x) * (x))
#endif

// GPU-friendly obstacle representation (POD type)
struct ObstacleGPU {
  float x, y;           // Center position
  float width, length;  // Dimensions
  float heading;        // Orientation in radians
  float type;           // ObstacleType as float for GPU compatibility
  float is_static;      // 1.0f for static, 0.0f for dynamic

  float3* future_traj;  // Device pointer to predicted trajectory [x, y, heading]
  int traj_len;         // Number of future trajectory points
  int reserved;         // Padding for alignment
};

// GPU-friendly obstacle list
struct ObstacleListGPU {
  ObstacleGPU* obstacles;  // Device pointer to obstacle array
  int count;               // Number of obstacles
};

struct TrajectoryCostParams : public CostParams<2> {
  float bike_position_coeff = 3000;
  float bike_velocity_coeff = 500;
  float bike_angle_coeff = 5000;
  float accel_effort_coeff = 100.0f;
  float steer_effort_coeff = 3000.0f;
  float target_velocity = 10.0f;
  float max_accel = 5;
  float min_accel = -5;
  float max_steer_angel = 0.20;
  float min_steer_angel = -0.20;
  float vehicle_width = 2.0f;
  float vehicle_length = 4.5f;
  float vehicle_wheelbase = 2.5f;
  float safety_margin = 0.8f;

  // Device-side waypoints data
  float3* waypoints = nullptr;  // Device pointer to waypoints array
  int waypoints_count = 0;      // Number of waypoints

  // Cache for temporal continuity optimization
  int last_matched_idx = 0;  // Last matched waypoint index for warm start

  // Device-side obstacle data (structured format)
  ObstacleListGPU obstacle_list;

  TrajectoryCostParams() {
    this->control_cost_coeff[0] = 20.0;
    this->control_cost_coeff[1] = 100.0;
  }
};

class TrajectoryCost : public Cost<TrajectoryCost, TrajectoryCostParams, VehicleDynamicsParams> {
public:
  TrajectoryCost(cudaStream_t stream = 0);
  ~TrajectoryCost();

  // Set waypoints from host (will be copied to device)
  void setWaypoints(const std::shared_ptr<ReferenceLine>& waypoints);

  // Set obstacles from host (will be copied to device)
  void setObstacles(const std::shared_ptr<common::ObstacleList>& obstacle_list);

  // Update matched index cache based on current vehicle position
  // Call this at the start of each planning cycle to initialize the cache
  void updateMatchedIndex(float x, float y);

  // Override paramsToDevice to handle waypoints GPU memory
  void paramsToDevice();

  // Override freeCudaMem to cleanup waypoints
  void freeCudaMem();

  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr,
                                    int* crash_status = nullptr);

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0,
                         int* crash_status = nullptr);

  __device__ float terminalCost(float* s, float* theta_c);

  float terminalCost(const Eigen::Ref<const output_array> s);

  float computeControlCost(const Eigen::Ref<const control_array> u, int timestep, int* crash);

  __device__ float computeControlCost(float* u, int timestep, float* theta_c, int* crash);

protected:
  std::shared_ptr<ReferenceLine> host_waypoints_;  // Keep host copy for updates
  float3* device_waypoints_ = nullptr;             // Device pointer for waypoints
  mutable int host_last_matched_idx_ = 0;          // Host-side cache for last matched index

  std::shared_ptr<common::ObstacleList> host_obstacles_;  // Keep host copy for updates
  ObstacleGPU* device_obstacles_ = nullptr;               // Device pointer for obstacle array
  std::vector<float3*> device_trajectories_;  // Device pointers for each obstacle's trajectory

  // Helper function to compute distance to reference line (host version)
  // Returns distance and optionally outputs nearest index and matched heading
  float computeDistanceToReferenceLine(float x, float y, int* nearest_idx = nullptr,
                                       float* matched_heading = nullptr) const;

  // Helper function to compute distance to reference line (device version)
  // Returns distance and outputs matched heading through reference parameter
  __device__ float computeDistanceToReferenceLineDevice(float x, float y,
                                                        float& matched_heading) const;

  // Common cost computation logic (host version)
  float computeCostInternal(float x, float y, float velocity, float heading, float accel,
                            float steer) const;

  // Common cost computation logic (device version)
  __device__ float computeCostInternalDevice(float x, float y, float velocity, float heading,
                                             float accel, float steer) const;

  // Helper function to check collision with obstacles (host version)
  float computeObstacleCost(float x, float y, float heading, float vehicle_width,
                            float vehicle_length) const;

  // Helper function to check collision with obstacles (device version)
  __device__ float computeObstacleCostDevice(float x, float y, float heading, float vehicle_width,
                                             float vehicle_length) const;
};

#ifdef __CUDACC__
inline __device__ float TrajectoryCost::computeDistanceToReferenceLineDevice(
    float x, float y, float& matched_heading) const {
  if (params_.waypoints == nullptr || params_.waypoints_count == 0) {
    return 0.0f;
  }

  const int n = params_.waypoints_count;
  const int cached_idx = params_.last_matched_idx;
  float min_dist = 1e6f;
  int min_idx = cached_idx;

  // Local search window size (tune based on your dynamics and dt)
  // Assuming max velocity ~30m/s and dt=0.1s, vehicle moves ~3m per step
  // With waypoint spacing ~0.2m, need to check ~15 points forward
  const int local_window = 30;  // Search ±30 points around cached_idx

  // Local search: only check nearby points for temporal continuity
  int search_start = max(0, cached_idx - 5);
  int search_end = min(n - 1, cached_idx + local_window);
  if (cached_idx == 0) {
    // If no cache, search the whole range
    search_start = 0;
    search_end = n - 1;
  }

  for (int i = search_start; i <= search_end; ++i) {
    float dx = x - params_.waypoints[i].x;
    float dy = y - params_.waypoints[i].y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  // Check line segment projection for more accurate distance
  if (min_idx > 0 && min_idx < n - 1) {
    // Segment [min_idx-1, min_idx]
    float x0 = params_.waypoints[min_idx - 1].x;
    float y0 = params_.waypoints[min_idx - 1].y;
    float x1 = params_.waypoints[min_idx].x;
    float y1 = params_.waypoints[min_idx].y;

    float dx_seg = x1 - x0;
    float dy_seg = y1 - y0;
    float seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg;

    if (seg_len_sq > 1e-6f) {
      float t = ((x - x0) * dx_seg + (y - y0) * dy_seg) / seg_len_sq;
      t = fmaxf(0.0f, fminf(1.0f, t));

      float proj_x = x0 + t * dx_seg;
      float proj_y = y0 + t * dy_seg;
      float dist = sqrtf((x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y));
      min_dist = fminf(min_dist, dist);
    }

    // Segment [min_idx, min_idx+1]
    x0 = x1;
    y0 = y1;
    x1 = params_.waypoints[min_idx + 1].x;
    y1 = params_.waypoints[min_idx + 1].y;

    dx_seg = x1 - x0;
    dy_seg = y1 - y0;
    seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg;

    if (seg_len_sq > 1e-6f) {
      float t = ((x - x0) * dx_seg + (y - y0) * dy_seg) / seg_len_sq;
      t = fmaxf(0.0f, fminf(1.0f, t));

      float proj_x = x0 + t * dx_seg;
      float proj_y = y0 + t * dy_seg;
      float dist = sqrtf((x - proj_x) * (x - proj_x) + (y - proj_y) * (y - proj_y));
      min_dist = fminf(min_dist, dist);
    }
  }

  // Update cache for next timestep (non-const operation on mutable params)
  // Note: This is safe in device code as each thread has its own trajectory
  const_cast<TrajectoryCostParams&>(params_).last_matched_idx = min_idx;

  matched_heading = params_.waypoints[min_idx].z;

  return min_dist;
}

inline __device__ float TrajectoryCost::computeCostInternalDevice(float x, float y, float velocity,
                                                                  float heading, float accel,
                                                                  float steer) const {
  float matched_heading = 0.0f;
  float lateral_distance = computeDistanceToReferenceLineDevice(x, y, matched_heading);

  float violation = 0.0f;
  if (accel >= params_.max_accel || accel <= params_.min_accel ||
      steer >= params_.max_steer_angel || steer <= params_.min_steer_angel || velocity < 0) {
    violation = 500;
  }

  // Compute heading error (normalize to [-pi, pi])
  float heading_error = heading - matched_heading;
  if (heading_error > M_PI_F) {
    heading_error -= M_2PI_F;
  } else if (heading_error < -M_PI_F) {
    heading_error += M_2PI_F;
  }

  // Compute obstacle collision cost (assuming vehicle dimensions)
  const float vehicle_width = 2.0f;   // TODO: make this configurable
  const float vehicle_length = 4.5f;  // TODO: make this configurable
  const float obstacle_cost =
      computeObstacleCostDevice(x, y, heading, vehicle_width, vehicle_length);

  const float position_cost = params_.bike_position_coeff * lateral_distance;
  const float velocity_cost =
      params_.bike_velocity_coeff * fabsf(velocity - params_.target_velocity);
  const float angle_cost = params_.bike_angle_coeff * fabsf(heading_error);
  const float accel_cost = params_.accel_effort_coeff * fabsf(accel);
  const float steer_cost = params_.steer_effort_coeff * fabsf(steer);
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost + obstacle_cost +
         violation;
}

inline __device__ float TrajectoryCost::computeStateCost(float* s, int timestep, float* theta_c,
                                                         int* crash_status) {
  (void)timestep;
  (void)theta_c;
  (void)crash_status;

  return computeCostInternalDevice(s[0], s[1], s[2], s[3], s[4], s[5]);
}

inline __device__ float TrajectoryCost::terminalCost(float* s, float* theta_c) {
  (void)theta_c;
  return computeCostInternalDevice(s[0], s[1], s[2], s[3], s[4], s[5]);
}

inline __device__ float TrajectoryCost::computeControlCost(float* u, int timestep, float* theta_c,
                                                           int* crash) {
  return params_.control_cost_coeff[0] * fabsf(u[0]) + params_.control_cost_coeff[1] * fabsf(u[1]);
}

inline __device__ float TrajectoryCost::computeObstacleCostDevice(float x, float y, float heading,
                                                                  float vehicle_width,
                                                                  float vehicle_length) const {
  if (params_.obstacle_list.obstacles == nullptr || params_.obstacle_list.count == 0) {
    return 0.0f;
  }

  float total_cost = 0.0f;

  // Iterate through all obstacles
  for (int i = 0; i < params_.obstacle_list.count; ++i) {
    const ObstacleGPU& obs = params_.obstacle_list.obstacles[i];

    float dist = sqrtf(SQ(x - obs.x) + SQ(y - obs.y));
    if (dist < 3.0f) {
      total_cost += 1000.0f;
      if (y > 0) {
        total_cost -= 700.0f;  // Bias to encourage passing on one side
      }
    }

    // Future trajectory cost (if available)
    if (obs.future_traj != nullptr && obs.traj_len > 0) {
      // TODO: Add temporal collision checking with predicted trajectory
      // This allows checking collision at future timesteps along the prediction horizon
    }
  }

  return total_cost;
}

#endif  // __CUDACC__
