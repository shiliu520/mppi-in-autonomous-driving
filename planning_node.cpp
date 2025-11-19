/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:59:20
 * @LastEditors: puyu yu.pu@qq.com
 * @LastEditTime: 2025-11-19 23:46:15
 * @FilePath: /mppi-in-autonomous-driving/planning_node.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "planning/stochastic_optimizer.cuh"
#include "simulator/simulator.hpp"

#include <csignal>

int main(int argc, char** argv) {
  StochasticOptimizer optimizer;
  std::atomic_bool done = false;
  static std::function<void()> sigint_handler = [&] { done = true; };
  std::signal(SIGINT, [](int) {
    if (sigint_handler) {
      sigint_handler();
      std::cout << "SIGINT received, shutting down..." << std::endl;
    }
  });

  Simulator simulator;
  simulator.start();

  const auto main_thread_start = std::chrono::steady_clock::now();
  auto next_tick = main_thread_start;
  while (!done) {
    auto ego_state = simulator.get_ego_state();
    auto control_input = optimizer.plan_once(ego_state);
    simulator.set_ego_control_input(control_input);

    next_tick += std::chrono::milliseconds(100);
    std::this_thread::sleep_until(next_tick);
  }

  simulator.stop();

  return 0;
}
