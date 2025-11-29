/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-23 15:30:39
 * @LastEditTime: 2025-11-23 21:27:33
 * @FilePath: /mppi-in-autonomous-driving/common/reference_line.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "reference_line.hpp"

#include <spdlog/spdlog.h>

ReferenceLine::ReferenceLine(std::vector<double> x_seq, std::vector<double> y_seq,
                             double accuracy /* = 0.2*/)
    : delta_s_(accuracy) {
  spline_ = CubicSpline2D(x_seq, y_seq);
  for (double s = 0.0; s <= spline_.s.back(); s += delta_s_) {
    Eigen::Vector2d pos = spline_.calc_position(s);
    double lyaw = spline_.calc_yaw(s);
    double lx = pos.x();
    double ly = pos.y();
    x_.emplace_back(lx);
    y_.emplace_back(ly);
    yaw_.emplace_back(lyaw);
    longitude_.emplace_back(s);
  }
}

Eigen::Vector3d ReferenceLine::calc_position(double cur_s) {
  Eigen::Vector2d pos = spline_.calc_position(cur_s);
  double lyaw = spline_.calc_yaw(cur_s);
  double lx = pos.x();
  double ly = pos.y();

  return {lx, ly, lyaw};
}

Eigen::Vector3d ReferenceLine::operator[](size_t index) const {
  if (index >= x_.size() || index >= y_.size() || index >= yaw_.size()) {
    throw std::out_of_range("Index out of range");
  }

  return Eigen::Vector3d{x_[index], y_[index], yaw_[index]};
}

Eigen::Vector3d ReferenceLine::at(size_t index) const {
  if (index >= x_.size() || index >= y_.size() || index >= yaw_.size()) {
    throw std::out_of_range("Index out of range");
  }

  return Eigen::Vector3d{x_[index], y_[index], yaw_[index]};
}
