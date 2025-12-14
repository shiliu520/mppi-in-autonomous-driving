/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-12-09 23:01:19
 * @LastEditTime: 2025-12-13 21:46:09
 * @FilePath: /mppi-in-autonomous-driving/simulator/simple_predictor.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common/common.hpp"
#include "commonroad_cpp/obstacle/obstacle.h"

#include <memory>
#include <vector>

std::vector<PathPoint> get_simple_prediction(const std::shared_ptr<Obstacle>& obstacle,
                                             size_t horizon_length);