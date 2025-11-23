/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-20 23:07:24
 * @LastEditTime: 2025-11-20 23:14:26
 * @FilePath: /mppi-in-autonomous-driving/simulator/cubic_spline.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once
#ifndef __CUBIC_SPLINE_HPP
#define __CUBIC_SPLINE_HPP

#include <Eigen/Core>
#include <vector>

namespace cubic_spline_utils {

template <typename T>
std::vector<T> vector_diff(const std::vector<T>& vec) {
  std::vector<T> ret;
  for (size_t idx = 1; idx < vec.size(); ++idx) {
    ret.push_back(vec[idx] - vec[idx - 1]);
  }
  return ret;
}

template <typename T>
std::vector<T> vector_cumsum(std::vector<T> vec) {
  std::vector<T> output;
  T tmp = 0;
  for (size_t idx = 0; idx < vec.size(); ++idx) {
    tmp += vec[idx];
    output.push_back(tmp);
  }
  return output;
}

}  // namespace cubic_spline_utils

class CubicSpline {
private:
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
  std::vector<double> h;
  int nx;

public:
  CubicSpline() {}
  CubicSpline(std::vector<double> _x, std::vector<double> _y);
  ~CubicSpline() {}
  Eigen::MatrixXd calc_A(void);
  Eigen::VectorXd calc_B(void);
  double calc_position(double _x) const;
  double calc_first_derivative(double _x) const;
  double calc_second_derivative(double _x) const;
  double operator()(double _x, int dd = 0) const;
};

class CubicSpline2D {
private:
  CubicSpline sx;
  CubicSpline sy;

public:
  std::vector<double> s;

  CubicSpline2D() {}
  CubicSpline2D(std::vector<double> _x, std::vector<double> _y);
  ~CubicSpline2D() {}
  std::vector<double> calc_s(std::vector<double> _x, std::vector<double> _y);
  Eigen::Vector2d calc_position(double _s) const;
  double calc_yaw(double _s) const;
  double calc_curvature(double _s) const;
  Eigen::Vector2d operator()(double _s, int n = 0) const;

  static std::vector<std::vector<double>> calc_spline_course(std::vector<double> x,
                                                             std::vector<double> y,
                                                             double ds = 0.1);
};

#endif
