/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-17 23:21:08
 * @LastEditTime: 2025-11-20 23:10:50
 * @FilePath: /mppi-in-autonomous-driving/planning/vehicle_dynamics.cuh
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "mppi/dynamics/dynamics.cuh"
#include "mppi/utils/math_utils.h"

struct VehicleDynamicsParams : public DynamicsParams {
  enum class StateIndex : int { POS_X = 0, POS_Y, VELOCITY, YAW, ACCEL, STEER, NUM_STATES };
  enum class ControlIndex : int { JERK = 0, STEER_RATE, NUM_CONTROLS };
  enum class OutputIndex : int { POS_X = 0, POS_Y, VELOCITY, YAW, ACCEL, STEER, NUM_OUTPUTS };
  float wheelbase = 2.5f;

  VehicleDynamicsParams() = default;
  VehicleDynamicsParams(float wheelbase) : wheelbase(wheelbase) {};
};

using namespace MPPI_internal;

class VehicleDynamics : public Dynamics<VehicleDynamics, VehicleDynamicsParams> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PARENT_CLASS = Dynamics<VehicleDynamics, VehicleDynamicsParams>;
  VehicleDynamics(float wheelbase = 2.5f, cudaStream_t stream = 0);

  std::string getDynamicsModelName() const override { return "Bicycle Model"; }

  void computeDynamics(const Eigen::Ref<const state_array>& state,
                       const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  bool computeGrad(const Eigen::Ref<const state_array>& state,
                   const Eigen::Ref<const control_array>& control, Eigen::Ref<dfdx> A,
                   Eigen::Ref<dfdu> B);
  __host__ __device__ float getWheelbase() const { return this->params_.wheelbase; }

  void printState(const Eigen::Ref<const state_array>& state);
  void printState(float* state);
  void printParams();

  __device__ void computeDynamics(float* state, float* control, float* state_der,
                                  float* theta_s = nullptr);

  state_array stateFromMap(const std::map<std::string, float>& map) override;
};

#ifdef __CUDACC__
inline __device__ void VehicleDynamics::computeDynamics(float* state, float* control,
                                                        float* state_der, float* theta_s) {
  (void)theta_s;

  const float theta = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  const float delta = angle_utils::normalizeAngle(state[S_INDEX(STEER)]);
  state_der[S_INDEX(POS_X)] = state[S_INDEX(VELOCITY)] * __cosf(theta);
  state_der[S_INDEX(POS_Y)] = state[S_INDEX(VELOCITY)] * __sinf(theta);
  state_der[S_INDEX(VELOCITY)] = state[S_INDEX(ACCEL)];
  state_der[S_INDEX(YAW)] =
      (state[S_INDEX(VELOCITY)] * __tanf(delta)) / this->params_.wheelbase;
  state_der[S_INDEX(ACCEL)] = control[C_INDEX(JERK)];
  state_der[S_INDEX(STEER)] = control[C_INDEX(STEER_RATE)];
}
#endif  // __CUDACC__

