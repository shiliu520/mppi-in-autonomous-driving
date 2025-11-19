#pragma once

#include <cmath>

#include "mppi/utils/file_utils.h"
#include "mppi/cost_functions/cost.cuh"
#include "vehicle_dynamics.cuh"

struct TrajectoryCostParams : public CostParams<2>
{
  float bike_position_coeff = 30000;
  float bike_velocity_coeff = 5000;
  float bike_angle_coeff = 20000;
  float accel_effort_coeff = 1000.0f;
  float steer_effort_coeff = 10000.0f;
  float target_velocity = 10.0f;

  TrajectoryCostParams()
  {
    this->control_cost_coeff[0] = 100.0;
    this->control_cost_coeff[1] = 500.0;
  }
};

class TrajectoryCost : public Cost<TrajectoryCost, TrajectoryCostParams, VehicleDynamicsParams>
{
public:
  TrajectoryCost(cudaStream_t stream = 0);

  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);

  __device__ float terminalCost(float* s, float* theta_c);

  float terminalCost(const Eigen::Ref<const output_array> s);

protected:
};

#ifdef __CUDACC__
inline __device__ float TrajectoryCost::computeStateCost(float* s, int timestep, float* theta_c,
                                                        int* crash_status) {
  (void)timestep;
  (void)theta_c;
  (void)crash_status;

  const float position_cost = params_.bike_position_coeff * fabsf(s[1]);
  const float velocity_cost =
      params_.bike_velocity_coeff * fabsf(s[2] - params_.target_velocity);
  const float angle_cost = params_.bike_angle_coeff * fabsf(s[3]);
  const float accel_cost = params_.accel_effort_coeff * fabsf(s[4]);
  const float steer_cost = params_.steer_effort_coeff * fabsf(s[5]);
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost;
}

inline __device__ float TrajectoryCost::terminalCost(float* s, float* theta_c) {
  (void)theta_c;

  const float position_cost = params_.bike_position_coeff * fabsf(s[1]);
  const float velocity_cost =
      params_.bike_velocity_coeff * fabsf(s[2] - params_.target_velocity);
  const float angle_cost = params_.bike_angle_coeff * fabsf(s[3]);
  const float accel_cost = params_.accel_effort_coeff * fabsf(s[4]);
  const float steer_cost = params_.steer_effort_coeff * fabsf(s[5]);
  return position_cost + velocity_cost + angle_cost + accel_cost + steer_cost;
}
#endif  // __CUDACC__

