/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:04:19
 * @LastEditTime: 2025-12-11 00:11:12
 * @FilePath: /mppi-in-autonomous-driving/simulator/simulator_utils.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "foxglove/schemas.hpp"
#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"

#include <cmath>
#include <vector>
#include <tuple>
#include <memory>

class Lanelet;
class RoadNetwork;

constexpr double kObstacleHeightVRU = 1.7;
constexpr double kObstacleHeightVehicle = 1.6;
constexpr double kObstacleHeightBus = 3.0;
constexpr foxglove::schemas::Color kObstacleColorVRU = {0.22, 0.43, 0.64, 0.8};      // #3A6EA5
constexpr foxglove::schemas::Color kObstacleColorBus = {0.8, 0.2, 0.2, 0.8};         // #CC3333
constexpr foxglove::schemas::Color kObstacleColorVehicle = {0.83, 0.59, 0.25, 0.8};  // #D4963F

inline foxglove::schemas::Quaternion yaw_to_quaternion(double yaw) {
  foxglove::schemas::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

inline foxglove::schemas::Pose construct_pose(void) {
  foxglove::schemas::Pose pose;
  pose.position = foxglove::schemas::Vector3{0.0, 0.0, 0.0};
  pose.orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0};
  return pose;
}

std::tuple<foxglove::schemas::Vector3, foxglove::schemas::Color> get_obstacle_size_and_color(
    ObstacleType obstacle_type, double obstacle_length, double obstacle_width);

/**
 * @brief Create a line using mesh triangulation to avoid transparency issues
 *
 * This function creates a TriangleListPrimitive representing a line with given thickness.
 * Using mesh triangulation avoids the "sugar-coated haws" effect (darker color near points)
 * that occurs with LinePrimitive when transparency is less than 1.
 *
 * @param points Vector of 3D points defining the line path
 * @param thickness Line thickness (width perpendicular to line direction)
 * @param color Line color (RGBA)
 * @param gradient_fade If true, alpha fades from start to 0 with non-linear decay
 * @return foxglove::schemas::TriangleListPrimitive Triangle mesh representing the line
 */
foxglove::schemas::TriangleListPrimitive create_line_mesh(
    const std::vector<foxglove::schemas::Point3>& points, double thickness,
    const foxglove::schemas::Color& color, bool gradient_fade = false);

/**
 * @brief Check if a lanelet is at the leftmost or rightmost edge of the road
 * 
 * This function determines if a lanelet is at the road edge by checking:
 * 1. Adjacent lanelet existence in the same direction
 * 2. Line marking type (curb, solid, etc.)
 * 3. Recursive check through all same-direction adjacent lanelets
 * 
 * @param lanelet The lanelet to check
 * @param road_network The road network containing the lanelet
 * @return std::pair<bool, bool> First: is leftmost, Second: is rightmost
 */
std::pair<bool, bool> is_lanelet_at_road_edge(
    const std::shared_ptr<Lanelet>& lanelet,
    const std::shared_ptr<RoadNetwork>& road_network);

/**
 * @brief Determine whether to draw left or right border lines for a lanelet
 * 
 * In merge/diverge scenarios, this function intelligently decides which border lines
 * should be drawn to avoid duplicate or missing lines:
 * - In merge scenarios: leftmost lane draws left line, rightmost draws right line
 * - In normal scenarios: draw lines based on road edge detection
 * - Shared borders between adjacent lanes are drawn only once
 * 
 * @param lanelet The lanelet to check
 * @param road_network The road network containing the lanelet
 * @return std::pair<bool, bool> First: should draw left line, Second: should draw right line
 */
std::pair<bool, bool> should_draw_lanelet_borders(
    const std::shared_ptr<Lanelet>& lanelet,
    const std::shared_ptr<RoadNetwork>& road_network);
