/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-26 23:54:17
 * @LastEditTime: 2025-11-27 22:17:58
 * @FilePath: /mppi-in-autonomous-driving/common/obstacle.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "common.hpp"

#include <string>
#include <vector>

namespace common {

enum class ObstacleType { VEHICLE = 0, PEDESTRIAN, BICYCLE, BUS, TRUCK, UNKNOWN };

class Obstacle {
public:
  Obstacle() = default;
  Obstacle(const std::string& id, double x, double y, double heading, double width, double length,
           bool is_static, ObstacleType type)
      : id_(id),
        x_(x),
        y_(y),
        heading_(heading),
        width_(width),
        length_(length),
        is_static_(is_static),
        type_(type) {}
  ~Obstacle() {}

  // getters and setters
  double x() const { return x_; }
  double y() const { return y_; }
  double width() const { return width_; }
  double length() const { return length_; }
  double heading() const { return heading_; }
  bool is_static() const { return is_static_; }
  ObstacleType type() const { return type_; }
  std::string id() const { return id_; }
  const std::vector<PathPoint>& prediction() const { return prediction_; }
  void set_x(double x) { x_ = x; }
  void set_y(double y) { y_ = y; }
  void set_heading(double heading) { heading_ = heading; }
  void set_width(double width) { width_ = width; }
  void set_length(double length) { length_ = length; }
  void set_id(const std::string& id) { id_ = id; }
  void set_is_static(bool is_static) { is_static_ = is_static; }
  void set_type(ObstacleType type) { type_ = type; }
  void set_prediction(const std::vector<PathPoint>& prediction) { prediction_ = prediction; }

private:
  std::string id_;
  double x_;
  double y_;
  double heading_;
  double width_;
  double length_;
  bool is_static_;
  ObstacleType type_;
  std::vector<PathPoint> prediction_;
};

class ObstacleList {
public:
  ObstacleList(/* args */) = default;
  ~ObstacleList() {}

  void append(const Obstacle& obstacle) { obstacles_.push_back(obstacle); }
  const std::vector<Obstacle>& obstacles() const { return obstacles_; }
  void set_obstacles(const std::vector<Obstacle>& obstacles) { obstacles_ = obstacles; }

private:
  /* data */
  std::vector<Obstacle> obstacles_;
};

}  // namespace common
