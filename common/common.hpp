/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 23:18:52
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-11-18 23:59:53
 * @FilePath: /mppi-in-autonomous-driving/common/common.hpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#pragma once

#include "foxglove/schemas.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cmath>
#include <iomanip>

#define LOG_TRACE(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::trace, __VA_ARGS__)
#define LOG_DEBUG(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::debug, __VA_ARGS__)
#define LOG_INFO(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::info, __VA_ARGS__)
#define LOG_WARN(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::warn, __VA_ARGS__)
#define LOG_ERROR(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::err, __VA_ARGS__)
#define LOG_CRITICAL(logger, ...) SPDLOG_LOGGER_CALL(logger, spdlog::level::critical, __VA_ARGS__)

struct ControlInput {
  double accel{0};  // longitudinal acceleration (m/s^2)
  double steer{0};  // front wheel steering angle (rad)
};

struct StateInfo {
  double x{0};         // position x (m)
  double y{0};         // position y (m)
  double velocity{0};  // velocity (m/s)
  double yaw{0};       // heading angle (rad)
  double accel{0};     // acceleration (m/s^2)
  double steer{0};     // front wheel steering angle (rad)

  foxglove::schemas::Pose to_pose() const {
    const double half_yaw = 0.5 * yaw;
    foxglove::schemas::Pose pose;
    pose.position = foxglove::schemas::Vector3{x, y, 0.0};
    pose.orientation = foxglove::schemas::Quaternion{0.0, 0.0, sin(half_yaw), cos(half_yaw)};
    return pose;
  }
};

struct VehicleInfo {
  double width;
  double length;
  double wheel_base;
};

class TimeUtil {
public:
  static double NowSeconds() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    return duration_cast<microseconds>(duration).count() / 1e6;
  }

  static foxglove::schemas::Timestamp NowTimestamp() {
    using namespace std::chrono;
    auto now = system_clock::now();
    auto duration = now.time_since_epoch();
    auto sec = duration_cast<seconds>(duration).count();
    auto nsec = duration_cast<nanoseconds>(duration).count() % 1000000000;
    foxglove::schemas::Timestamp ts;
    ts.sec = static_cast<uint32_t>(sec);
    ts.nsec = static_cast<uint32_t>(nsec);
    return ts;
  }

  static std::string NowTimeString() {
    using namespace std::chrono;
    auto tp = system_clock::now();
    auto t = system_clock::to_time_t(tp);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    return oss.str();
  }
};

/**
 * @brief Template function to keep a fixed number of decimal places for floating-point numbers
 * @tparam digits Number of decimal places to keep, limited to 8 if exceeds
 * @param value Input floating-point number
 * @return double Floating-point number with specified decimal places
 */
template <uint8_t digits>
constexpr double to_fixed(double value) {
  constexpr int actual_digits = (digits > 8) ? 8 : digits;
  constexpr double multiplier = std::pow(10.0, actual_digits);
  return std::round(value * multiplier) / multiplier;
}
