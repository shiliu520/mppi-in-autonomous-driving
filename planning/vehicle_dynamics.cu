#include "mppi/utils/math_utils.h"

#include "vehicle_dynamics.cuh"

VehicleDynamics::VehicleDynamics(float wheelbase, cudaStream_t stream)
    : Dynamics<VehicleDynamics, VehicleDynamicsParams>(stream) {
  this->params_.wheelbase = wheelbase;
}

bool VehicleDynamics::computeGrad(const Eigen::Ref<const state_array>& state,
                                  const Eigen::Ref<const control_array>& control,
                                  Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B) {
  A.setZero();
  B.setZero();

  const float theta = state(S_INDEX(YAW));
  const float delta = state(S_INDEX(STEER));
  A(0, 2) = cosf(theta);
  A(0, 3) = -state(S_INDEX(VELOCITY)) * sinf(theta);
  A(1, 2) = sinf(theta);
  A(1, 3) = state(S_INDEX(VELOCITY)) * cosf(theta);
  A(2, 4) = 1;
  A(3, 2) = tanf(delta) / this->params_.wheelbase;
  A(3, 5) = state(S_INDEX(VELOCITY)) / (this->params_.wheelbase * SQ(cosf(delta)));
  B(4, 0) = 1;
  B(5, 1) = 1;

  return true;
}

void VehicleDynamics::computeDynamics(const Eigen::Ref<const state_array>& state,
                                      const Eigen::Ref<const control_array>& control,
                                      Eigen::Ref<state_array> state_der) {
  const float theta = angle_utils::normalizeAngle(state[S_INDEX(YAW)]);
  const float delta = angle_utils::normalizeAngle(state[S_INDEX(STEER)]);

  state_der(S_INDEX(POS_X)) = state(S_INDEX(VELOCITY)) * cosf(theta);
  state_der(S_INDEX(POS_Y)) = state(S_INDEX(VELOCITY)) * sinf(theta);
  state_der(S_INDEX(VELOCITY)) = state(S_INDEX(ACCEL));
  state_der(S_INDEX(YAW)) = (state(S_INDEX(VELOCITY)) * tanf(delta)) / this->params_.wheelbase;
  state_der(S_INDEX(ACCEL)) = control(C_INDEX(JERK));
  state_der(S_INDEX(STEER)) = control(C_INDEX(STEER_RATE));
}

Dynamics<VehicleDynamics, VehicleDynamicsParams>::state_array VehicleDynamics::stateFromMap(
    const std::map<std::string, float>& map) {
  state_array state;
  state.setZero();
  state(S_INDEX(POS_X)) = map.at("POS_X");
  state(S_INDEX(POS_Y)) = map.at("POS_Y");
  state(S_INDEX(VELOCITY)) = map.at("VELOCITY");
  state(S_INDEX(YAW)) = map.at("YAW");
  state(S_INDEX(ACCEL)) = map.at("ACCEL");
  state(S_INDEX(STEER)) = map.at("STEER");
  return state;
}

void VehicleDynamics::printState(const Eigen::Ref<const state_array>& state) {
  printf("%f %f %f %f %f %f \n", state(0), state(1), state(2), state(3), state(4), state(5));
}

void VehicleDynamics::printState(float* state) {
  printf("%f %f %f %f %f %f \n", state[0], state[1], state[2], state[3], state[4], state[5]);
}
