/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-23 15:30:47
 * @LastEditTime: 2025-11-23 21:19:56
 * @FilePath: /mppi-in-autonomous-driving/common/reference_line.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "cubic_spline.hpp"

#include <Eigen/Core>
#include <vector>

class ReferenceLine {
public:
  ReferenceLine() = delete;
  ReferenceLine(std::vector<double> _x, std::vector<double> _y, double accuracy = 0.2);
  ~ReferenceLine() {}

  Eigen::Vector3d calc_position(double cur_s);
  Eigen::Vector3d operator[](size_t index) const;
  Eigen::Vector3d at(size_t index) const;
  size_t size() const { return x_.size(); }
  double length() const { return spline_.s.back(); }
  double accuracy() const { return delta_s_; }

public:
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> yaw_;
  std::vector<double> longitude_;
  CubicSpline2D spline_;
  double delta_s_;
};
