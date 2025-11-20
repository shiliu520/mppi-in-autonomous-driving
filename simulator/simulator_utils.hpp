/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:04:19
 * @LastEditTime: 2025-11-20 23:09:51
 * @FilePath: /mppi-in-autonomous-driving/simulator/simulator_utils.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "foxglove/schemas.hpp"

#include <cmath>

inline foxglove::schemas::Quaternion yaw_to_quaternion(double yaw) {
  foxglove::schemas::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
