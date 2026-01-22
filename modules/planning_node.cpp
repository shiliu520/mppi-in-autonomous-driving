/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:59:20
 * @LastEditTime: 2026-01-21 00:23:28
 * @FilePath: /mppi-in-autonomous-driving/planning_node.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "modules/planner/stochastic_optimizer.cuh"
#include "modules/simulator/simulator.hpp"
#include "modules/visualizer/visualizer.hpp"

#include <csignal>
#include <getopt.h>
#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
  int opt;
  const char* optstring = "c:";
  std::string config_file_path;

  while ((opt = getopt(argc, argv, optstring)) != -1) {
    switch (opt) {
      case 'c':
        config_file_path = optarg;
        break;
      default:
        spdlog::error("Usage: {} [-c]", argv[0]);
        exit(EXIT_FAILURE);
    }
  }

  if (config_file_path.empty()) {
    spdlog::error("Usage: {} [-c]", argv[0]);
    exit(EXIT_FAILURE);
  }

  YAML::Node config;
  try {
    config = YAML::LoadFile(config_file_path);
  } catch (const YAML::Exception& e) {
    spdlog::error("Error parsing YAML file: {}", e.what());
    return 1;
  }

  std::atomic_bool done = false;
  static std::function<void()> sigint_handler = [&] { done = true; };
  std::signal(SIGINT, [](int) {
    if (sigint_handler) {
      sigint_handler();
      spdlog::info("SIGINT received, shutting down...");
    }
  });

  auto visualizer = std::make_shared<Visualizer>(config);
  Simulator simulator(config, visualizer);

  if (!visualizer->start()) {
    spdlog::info("Failed to start visualizer.");
    return 1;
  }

  auto run_simulation = [&](auto optimizer_ptr) {
    protos::config::SimulationConfig sim_config;
    optimizer_ptr->set_parameters_to_proto(&sim_config);
    auto [scenario_file_path, planning_problem_id] = simulator.get_planning_problem();
    sim_config.set_scenario_file_path(scenario_file_path);
    sim_config.set_problem_id(planning_problem_id);
    visualizer->log_simulation_config(sim_config);

    simulator.start();
    const auto main_thread_start = std::chrono::steady_clock::now();
    auto next_tick = main_thread_start;
    while (!done) {
      auto ego_state = simulator.get_ego_state();
      auto reference_line = simulator.get_reference_line();
      auto obstacle_list = simulator.get_obstacle_list();
      auto control_input = optimizer_ptr->plan_once(ego_state, reference_line, obstacle_list);
      optimizer_ptr->get_debug_result(ego_state);
      simulator.set_ego_control_input(control_input);

      next_tick += std::chrono::milliseconds(100);
      std::this_thread::sleep_until(next_tick);
    }
    simulator.stop();
    visualizer->stop();
  };

  int num_rollouts = config["planning"]["mppi_params"]["num_samples"].as<int>(8192);
  switch (num_rollouts) {
    case 1024: {
      auto optimizer = std::make_unique<StochasticOptimizer<1024>>(config, visualizer);
      run_simulation(std::move(optimizer));
      break;
    }
    case 2048: {
      auto optimizer = std::make_unique<StochasticOptimizer<2048>>(config, visualizer);
      run_simulation(std::move(optimizer));
      break;
    }
    case 4096: {
      auto optimizer = std::make_unique<StochasticOptimizer<4096>>(config, visualizer);
      run_simulation(std::move(optimizer));
      break;
    }
    case 8192: {
      auto optimizer = std::make_unique<StochasticOptimizer<8192>>(config, visualizer);
      run_simulation(std::move(optimizer));
      break;
    }
    case 16384: {
      auto optimizer = std::make_unique<StochasticOptimizer<16384>>(config, visualizer);
      run_simulation(std::move(optimizer));
      break;
    }
    default:
      spdlog::error("Error: Unsupported num_rollouts value: {}", num_rollouts);
      spdlog::error("Supported values are: 1024, 2048, 4096, 8192, 16384");
      return 1;
  }

  return 0;
}
