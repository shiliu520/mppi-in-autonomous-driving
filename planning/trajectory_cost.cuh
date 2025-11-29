/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:30:11
 * @LastEditTime: 2025-11-29 21:22:17
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

#define S_IDX(name) S_IND_CLASS(VehicleDynamicsParams, name)

struct VehicleCorners {
  float2 FL, FR, RL, RR;
};

// GPU-friendly obstacle representation (POD type)
struct ObstacleGPU {
  float x, y;           // Center position
  float width, length;  // Dimensions
  float heading;        // Orientation in radians
  int type;             // ObstacleType as float for GPU compatibility
  bool is_static;       // true for static, false for dynamic

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
  float position_coeff = 300;
  float velocity_coeff = 50;
  float angle_coeff = 500;
  float accel_effort_coeff = 10.0f;
  float steer_effort_coeff = 300.0f;
  float target_velocity = 10.0f;
  float max_accel = 5;
  float min_accel = -5;
  float max_steer_angle = 0.20;
  float min_steer_angle = -0.20;
  // vehicle dimensions and safety margin
  float vehicle_width = 1.85f;
  float vehicle_length = 4.85f;
  float wheelbase = 3.0f;
  float axle_to_front_bumper = 3.90f;
  float axle_to_rear_bumper = 0.95f;
  float longitudinal_safety_margin = 1.8f;
  float lateral_safety_margin = 0.6f;

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
  float computeCostInternal(const Eigen::Ref<const output_array> s) const;

  // Common cost computation logic (device version)
  __device__ float computeCostInternalDevice(float* state) const;

  // Helper function to check collision with obstacles (host version)
  float computeObstacleCost(float x, float y, float heading) const;

  // Helper function to check collision with obstacles (device version)
  __device__ float computeObstacleCostDevice(float x, float y, float heading) const;

  float computeSafetyMargin(const Eigen::Vector2f& ego_position,
                            const Eigen::Vector3f& obstacle_pose,
                            const Eigen::Vector2f& ellipse_axes) const;

  __device__ float computeSafetyMargin(float2 ego_position, float3 obstacle_pose,
                                       float2 ellipse_axes) const;

  // Shared host/device functions for obstacle collision detection
  __host__ __device__ VehicleCorners computeVehicleCorners(float3 ego_pose) const;
  __host__ __device__ bool pointInCapsule(float3 capsule_pose, float half_length, float half_width,
                                          float2 point) const;
  __host__ __device__ bool cornersInCapsule(const VehicleCorners& vehicle_corners,
                                            float3 capsule_pose, float half_length,
                                            float half_width) const;
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

  const int local_window = 10;
  // Local search: only check nearby points for temporal continuity
  int search_start = max(0, cached_idx - 2);
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

inline __device__ float TrajectoryCost::computeCostInternalDevice(float* state) const {
  float x = state[S_IDX(POS_X)];
  float y = state[S_IDX(POS_Y)];
  float velocity = state[S_IDX(VELOCITY)];
  float heading = state[S_IDX(YAW)];
  float accel = state[S_IDX(ACCEL)];
  float steer = state[S_IDX(STEER)];

  float matched_heading = 0.0f;
  float lateral_distance = computeDistanceToReferenceLineDevice(x, y, matched_heading);

  float violation = 0.0f;
  if (accel >= params_.max_accel || accel <= params_.min_accel ||
      steer >= params_.max_steer_angle || steer <= params_.min_steer_angle || velocity < -0.0001f) {
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
  const float obstacle_cost = computeObstacleCostDevice(x, y, heading);

  const float position_cost = params_.position_coeff * lateral_distance;
  const float velocity_cost = params_.velocity_coeff * fabsf(velocity - params_.target_velocity);
  const float angle_cost = params_.angle_coeff * fabsf(heading_error);
  const float accel_cost = params_.accel_effort_coeff * fabsf(accel);
  const float steer_cost = params_.steer_effort_coeff * fabsf(steer);
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost + obstacle_cost +
         violation;
}

inline __device__ float TrajectoryCost::computeStateCost(float* s, [[maybe_unused]] int timestep,
                                                         [[maybe_unused]] float* theta_c,
                                                         [[maybe_unused]] int* crash_status) {
  return computeCostInternalDevice(s);
}

inline __device__ float TrajectoryCost::terminalCost(float* s, [[maybe_unused]] float* theta_c) {
  return computeCostInternalDevice(s);
}

inline __device__ float TrajectoryCost::computeControlCost(float* u, int timestep, float* theta_c,
                                                           int* crash) {
  return params_.control_cost_coeff[0] * fabsf(u[0]) + params_.control_cost_coeff[1] * fabsf(u[1]);
}

inline __device__ float TrajectoryCost::computeSafetyMargin(float2 ego_position,
                                                            float3 obstacle_pose,
                                                            float2 ellipse_axes) const {
  const float diff_x = obstacle_pose.x - ego_position.x;
  const float diff_y = obstacle_pose.y - ego_position.y;
  const float cos_heading = __cosf(obstacle_pose.z);
  const float sin_heading = __sinf(obstacle_pose.z);

  // Rotate the difference into the obstacle's frame
  float rotated_x = diff_x * cos_heading + diff_y * sin_heading;
  float rotated_y = -diff_x * sin_heading + diff_y * cos_heading;
  // Compute normalized distance in ellipse frame
  float norm = (rotated_x * rotated_x) / (ellipse_axes.x * ellipse_axes.x) +
               (rotated_y * rotated_y) / (ellipse_axes.y * ellipse_axes.y);

  // Safety margin is how far inside the ellipse we are (positive if outside)
  return norm - 1.0f;
}

inline __host__ __device__ VehicleCorners
TrajectoryCost::computeVehicleCorners(float3 ego_pose) const {
  const float cos_h = cosf(ego_pose.z);
  const float sin_h = sinf(ego_pose.z);

  // Vehicle dimensions
  const float half_width = params_.vehicle_width / 2.0f;
  const float front_length = params_.axle_to_front_bumper;
  const float rear_length = params_.axle_to_rear_bumper;

  // Compute corner positions in vehicle frame, then transform to world frame
  VehicleCorners corners;

  // Front-left corner
  float fl_x_local = front_length;
  float fl_y_local = half_width;
  corners.FL.x = ego_pose.x + fl_x_local * cos_h - fl_y_local * sin_h;
  corners.FL.y = ego_pose.y + fl_x_local * sin_h + fl_y_local * cos_h;

  // Front-right corner
  float fr_x_local = front_length;
  float fr_y_local = -half_width;
  corners.FR.x = ego_pose.x + fr_x_local * cos_h - fr_y_local * sin_h;
  corners.FR.y = ego_pose.y + fr_x_local * sin_h + fr_y_local * cos_h;

  // Rear-left corner
  float rl_x_local = -rear_length;
  float rl_y_local = half_width;
  corners.RL.x = ego_pose.x + rl_x_local * cos_h - rl_y_local * sin_h;
  corners.RL.y = ego_pose.y + rl_x_local * sin_h + rl_y_local * cos_h;

  // Rear-right corner
  float rr_x_local = -rear_length;
  float rr_y_local = -half_width;
  corners.RR.x = ego_pose.x + rr_x_local * cos_h - rr_y_local * sin_h;
  corners.RR.y = ego_pose.y + rr_x_local * sin_h + rr_y_local * cos_h;

  return corners;
}

inline __host__ __device__ bool TrajectoryCost::pointInCapsule(float3 capsule_pose,
                                                               float half_length, float half_width,
                                                               float2 point) const {
  const float cos_h = cosf(capsule_pose.z);
  const float sin_h = sinf(capsule_pose.z);

  // Step 1: Construct line segment endpoints A, B
  float Ax = capsule_pose.x - half_length * cos_h;
  float Ay = capsule_pose.y - half_length * sin_h;
  float Bx = capsule_pose.x + half_length * cos_h;
  float By = capsule_pose.y + half_length * sin_h;

  // Step 2: AP and AB vectors
  float APx = point.x - Ax;
  float APy = point.y - Ay;

  float ABx = Bx - Ax;
  float ABy = By - Ay;

  float AB_len2 = ABx * ABx + ABy * ABy;

  // Projection parameter t
  float t = (APx * ABx + APy * ABy) / AB_len2;
  t = fmaxf(0.0f, fminf(1.0f, t));  // Clamp to [0, 1]

  // Closest point C on line segment to point P
  float Cx = Ax + t * ABx;
  float Cy = Ay + t * ABy;

  // Step 3: Check distance
  float dx = point.x - Cx;
  float dy = point.y - Cy;

  return (dx * dx + dy * dy) <= (half_width * half_width);
}

inline __host__ __device__ bool TrajectoryCost::cornersInCapsule(
    const VehicleCorners& vehicle_corners, float3 capsule_pose, float half_length,
    float half_width) const {
  // Check if any vehicle corner is inside the capsule
  if (pointInCapsule(capsule_pose, half_length, half_width, vehicle_corners.FL)) return true;
  if (pointInCapsule(capsule_pose, half_length, half_width, vehicle_corners.FR)) return true;
  if (pointInCapsule(capsule_pose, half_length, half_width, vehicle_corners.RL)) return true;
  if (pointInCapsule(capsule_pose, half_length, half_width, vehicle_corners.RR)) return true;
  return false;
}

inline __device__ float TrajectoryCost::computeObstacleCostDevice(float x, float y,
                                                                  float heading) const {
  if (params_.obstacle_list.obstacles == nullptr || params_.obstacle_list.count == 0) {
    return 0.0f;
  }

  float total_cost = 0.0f;
  float3 ego_pose{x, y, heading};

  VehicleCorners vehicle_corners = computeVehicleCorners(ego_pose);

  // Iterate through all obstacles
  for (int i = 0; i < params_.obstacle_list.count; ++i) {
    const ObstacleGPU& obs = params_.obstacle_list.obstacles[i];

    float3 capsule_pose{obs.x, obs.y, obs.heading};
    float half_length = obs.length / 2.0f + params_.longitudinal_safety_margin;
    float half_width = obs.width / 2.0f + params_.lateral_safety_margin;

    // Check if any vehicle corner is inside the obstacle capsule
    if (cornersInCapsule(vehicle_corners, capsule_pose, half_length, half_width)) {
      total_cost += 1000.0f;
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
