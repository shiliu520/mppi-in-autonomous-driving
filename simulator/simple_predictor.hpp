/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-12-09 23:01:19
 * @LastEditTime: 2025-12-09 23:14:25
 * @FilePath: /mppi-in-autonomous-driving/simulator/simple_predictor.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include <vector>
#include <memory>

#include "common/common.hpp"
#include "commonroad_cpp/obstacle/obstacle.h"

std::vector<PathPoint> get_simple_prediction(const std::shared_ptr<Obstacle>& obstacle,
                                             size_t horizon_length);