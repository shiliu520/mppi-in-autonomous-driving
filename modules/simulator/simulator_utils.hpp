/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:04:19
 * @LastEditTime: 2026-01-20 00:35:54
 * @FilePath: /mppi-in-autonomous-driving/modules/simulator/simulator_utils.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "commonroad_cpp/roadNetwork/road_network.h"

#include <memory>

/**
 * @brief Compute distances to left and right road edges from a given position
 *
 * This function finds the lanelet containing the position and computes the distances
 * to the actual left and right road edges, considering merge/diverge scenarios.
 * Uses cross product to determine the true leftmost and rightmost lanes in merge scenarios.
 *
 * @param px X coordinate of the position
 * @param py Y coordinate of the position
 * @param road_network The road network to search in
 * @return std::pair<double, double> First: left edge distance, Second: right edge distance
 *         Returns {20.0, 20.0} if position is not in any lanelet
 */
std::pair<double, double> compute_road_edge_distances(
    double px, double py, const std::shared_ptr<RoadNetwork>& road_network);
