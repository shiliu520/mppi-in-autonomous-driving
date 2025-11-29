/*
 * @Author: puyu yu.pu@qq.com
 * @Date: 2025-11-15 22:57:28
 * @LastEditTime: 2025-11-29 20:31:40
 * @FilePath: /mppi-in-autonomous-driving/simulator/simulator.cpp
 * Copyright (c) 2025 by puyu, All Rights Reserved.
 */

#include "common/common.hpp"
#include "common/cubic_spline.hpp"
#include "commonroad_cpp/interfaces/commonroad/input_utils.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "simulator.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <system_error>
#include <vector>

Simulator::Simulator() {
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  logger_ = std::make_shared<spdlog::logger>("simulator_logger", console_sink);
  logger_->set_level(spdlog::level::from_str("info"));
  logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^\033[1m%l\033[0m%$] [%s:%#] %v");

  save_mcap_ = true;
  vehicle_info_.wheel_base = 2.8;
  ego_state_.x = 0;
  ego_state_.y = 0;
  ego_state_.velocity = 0;
  ego_state_.heading = 0;
  ego_state_.accel = 0;
  ego_state_.steer = 0;

  // TODO: Load scenario from CommonRoad file, something hardcoded for testing now
  std::string pathToTestFileOne{
      "/home/puyu/codes/mppi-in-autonomous-driving/scenarios/static_test.xml"};
  const auto& [obstaclesScenarioOne, roadNetworkScenarioOne, timeStepSizeOne, planningProblems] =
      InputUtils::getDataFromCommonRoad(pathToTestFileOne);
  WorldParameters wp{RoadNetworkParameters(), SensorParameters(), ActuatorParameters::egoDefaults(),
                     TimeParameters(5, 0.3, 0.1), ActuatorParameters::vehicleDefaults()};
  std::vector<std::shared_ptr<Obstacle>> egos = {};
  sim_world_ = std::make_unique<World>("ARG_Carcarana-6_5_T-1", 0, roadNetworkScenarioOne, egos,
                                       obstaclesScenarioOne, timeStepSizeOne, wp);

  const std::shared_ptr<LaneletGraph>& topo_graph =
      sim_world_->getRoadNetwork()->getTopologicalMap();
  auto paths = topo_graph->findPaths(1, 22, true);
  if (!paths.empty()) {
    std::vector<double> ref_xs;
    std::vector<double> ref_ys;
    bool is_lane_change = false;
    for (size_t idx = 0; idx < paths.size(); ++idx) {
      auto lanelet = sim_world_->getRoadNetwork()->findLaneletById(paths[idx]);
      if (idx + 1 < paths.size()) {
        LOG_INFO(logger_, "Routing lanelet {} -> {}", paths[idx], paths[idx + 1]);
        auto next_lanelet = sim_world_->getRoadNetwork()->findLaneletById(paths[idx + 1]);
        if (lanelet_operations::areLaneletsAdjacent(lanelet, next_lanelet)) {
          is_lane_change = true;
          continue;
        }
      }
      for (const auto& vertex : lanelet->getCenterVertices()) {
        if (ref_xs.size() > 0) {
          const double last_x = ref_xs.back();
          const double last_y = ref_ys.back();
          const double dist =
              std::sqrt(std::pow(vertex.x - last_x, 2) + std::pow(vertex.y - last_y, 2));
          if (dist < 0.1) {
            continue;
          }
        }
        if (is_lane_change) {
          is_lane_change = false;
          continue;
        }
        ref_xs.push_back(vertex.x);
        ref_ys.push_back(vertex.y);
      }
    }

    routing_reference_line_ = std::make_shared<ReferenceLine>(ref_xs, ref_ys, 0.5);
    LOG_INFO(logger_, "Reference line created with {} points, length {:.2f} m",
             routing_reference_line_->size(), routing_reference_line_->length());
  } else {
    LOG_ERROR(logger_, "No path found from lanelet 2 to 22");
    routing_reference_line_ = nullptr;
  }
}

Simulator::~Simulator() { stop(); }

bool Simulator::start() {
  if (running_) {
    return true;
  }
  if (!register_publish_channels()) {
    LOG_ERROR(logger_, "Failed to register publish channels.");
    return false;
  }
  if (save_mcap_) {
    foxglove::McapWriterOptions mcap_options = {};
    std::filesystem::path mcap_save_dir = std::filesystem::path(PROJECT_SOURCE_DIR) / "logs";
    if (!std::filesystem::exists(mcap_save_dir)) {
      if (std::filesystem::create_directories(mcap_save_dir)) {
        LOG_INFO(logger_, "Created mcap save directory: {}", mcap_save_dir.string());
      } else {
        LOG_ERROR(logger_, "Failed to create mcap save directory: {}", mcap_save_dir.string());
        return false;
      }
    }
    std::string mcap_file_path = mcap_save_dir / ("mppi_" + TimeUtil::NowTimeString() + ".mcap");
    mcap_options.path = mcap_file_path;
    mcap_options.compression = foxglove::McapCompression::Lz4;
    auto writer_result = foxglove::McapWriter::create(mcap_options);
    if (!writer_result.has_value()) {
      LOG_ERROR(logger_, "Failed to create writer: {}", foxglove::strerror(writer_result.error()));
      return false;
    }
    mcap_writer_ = std::make_unique<foxglove::McapWriter>(std::move(writer_result.value()));
  }

  running_ = true;
  sim_thread_ = std::thread(&Simulator::simulation_loop, this);
  LOG_INFO(logger_, "Simulator started.");

  return true;
}

void Simulator::stop() {
  if (!running_) {
    return;
  }
  running_ = false;
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }
  LOG_INFO(logger_, "Simulator stopped.");
}

void Simulator::simulation_loop() {
  if (routing_reference_line_) {
    reference_line_channel_->log(get_reference_line_scene_update());
  }

  const auto loop_start = std::chrono::steady_clock::now();
  auto next_tick = loop_start;
  uint32_t loop_count = 0;
  while (running_) {
    update_ego_state();

    const auto ego_pose = get_ego_state().to_pose();
    const auto ego_car_scene_update = get_ego_scene_update();

    FrameTransform transform;
    transform.timestamp = TimeUtil::NowTimestamp();
    transform.parent_frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.translation = ego_pose.position;
    transform.rotation = ego_pose.orientation;

    auto dur = std::chrono::steady_clock::now() - loop_start;
    const double elapsed_seconds = std::chrono::duration<double>(dur).count();
    std::string elapsed_msg = "{\"elapsed\": " + std::to_string(elapsed_seconds) + "}";
    loop_runtime_channel_->log(reinterpret_cast<const std::byte*>(elapsed_msg.data()),
                               elapsed_msg.size());
    ego_car_channel_->log(ego_car_scene_update);
    transform_channel_->log(transform);
    if (planning_info_updated_.load()) {
      std::shared_lock lock(planning_info_mutex_);
      std::string debug_info_data = planning_info_.SerializeAsString();
      planning_info_channel_->log(reinterpret_cast<const std::byte*>(debug_info_data.data()),
                                  debug_info_data.size());
      const auto traj_scene_update = get_trajectory_scene_update();
      const auto sampled_scene_update = get_sampled_scene_update();
      trajectory_channel_->log(traj_scene_update);
      sampled_channel_->log(sampled_scene_update);
      planning_info_updated_.store(false);
    }

    if (loop_count % 250 == 0) {
      // publish lanelets scene at 0.2 Hz
      lanelet_scene_channel_->log(
          get_lanelets_scene_update(sim_world_->getRoadNetwork()->getLaneletNetwork()));
    }
    if (loop_count % 5 == 0) {
      // publish obstacles scene at 10 Hz
      obstacle_list_channel_->log(get_obstacle_list_scene_update());
    }

    ++loop_count;
    next_tick += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(next_tick);
  }
  if (mcap_writer_) {
    foxglove::FoxgloveError err = mcap_writer_->close();
    if (err != foxglove::FoxgloveError::Ok) {
      LOG_ERROR(logger_, "Failed to close writer: {}", foxglove::strerror(err));
    } else {
      LOG_INFO(logger_, "mcap file saved.");
    }
  }
}

StateInfo Simulator::get_ego_state() const {
  std::shared_lock lock(ego_state_mutex_);
  return ego_state_;
}

void Simulator::set_ego_control_input(const ControlInput& input) {
  std::unique_lock lock(ego_state_mutex_);
  ego_state_.accel = input.accel;
  ego_state_.steer = input.steer;
}

void Simulator::update_ego_state() {
  const double current_time_sec = TimeUtil::NowSeconds();
  if (last_ego_update_time_ > 0.1) {
    const double dt = current_time_sec - last_ego_update_time_;
    std::unique_lock lock(ego_state_mutex_);
    ego_state_.x += ego_state_.velocity * std::cos(ego_state_.heading) * dt;
    ego_state_.y += ego_state_.velocity * std::sin(ego_state_.heading) * dt;
    ego_state_.heading +=
        ego_state_.velocity * (std::tan(ego_state_.steer) / vehicle_info_.wheel_base) * dt;
    ego_state_.velocity += ego_state_.accel * dt;

    LOG_DEBUG(logger_,
              "update_ego_state >>> x: {:.2f}, y: {:.2f}, v: {:.2f}, heading: {:.5f} accel: {:.2f} "
              "steer: {:.5f}",
              ego_state_.x, ego_state_.y, ego_state_.velocity, ego_state_.heading, ego_state_.accel,
              ego_state_.steer);
  }
  last_ego_update_time_ = current_time_sec;
}

bool Simulator::register_publish_channels(void) {
  using foxglove::RawChannel;
  using foxglove::WebSocketServer;

  foxglove::setLogLevel(foxglove::LogLevel::Error);
  foxglove::WebSocketServerOptions ws_options;
  ws_options.host = "0.0.0.0";
  ws_options.port = 8765;

  auto server_result = WebSocketServer::create(std::move(ws_options));
  if (!server_result.has_value()) {
    std::cerr << foxglove::strerror(server_result.error()) << '\n';
    return false;
  }
  socket_server_ = std::make_unique<WebSocketServer>(std::move(server_result.value()));

  auto make_channel = [](auto&& result) {
    return std::make_unique<std::decay_t<decltype(result.value())>>(std::move(result.value()));
  };

  loop_runtime_channel_ = make_channel(RawChannel::create("/simulation/runtime_secs", "json"));
  ego_car_channel_ = make_channel(SceneUpdateChannel::create("/markers/ego_car"));
  lanelet_scene_channel_ = make_channel(SceneUpdateChannel::create("/markers/hdmap_lanelets"));
  trajectory_channel_ = make_channel(SceneUpdateChannel::create("/markers/trajectory"));
  sampled_channel_ = make_channel(SceneUpdateChannel::create("/markers/sampled_trajectories"));
  reference_line_channel_ = make_channel(SceneUpdateChannel::create("/markers/reference_line"));
  obstacle_list_channel_ = make_channel(SceneUpdateChannel::create("/markers/obstacles"));
  transform_channel_ = make_channel(FrameTransformChannel::create("/transform/map_to_baselink"));

  auto descriptor = planning::protos::PlanningInfo::descriptor();
  foxglove::Schema schema;
  schema.encoding = "protobuf";
  schema.name = descriptor->full_name();
  // Create a FileDescriptorSet containing our message descriptor
  google::protobuf::FileDescriptorSet file_descriptor_set;
  const google::protobuf::FileDescriptor* file_descriptor = descriptor->file();
  file_descriptor->CopyTo(file_descriptor_set.add_file());
  std::string serialized_descriptor = file_descriptor_set.SerializeAsString();
  schema.data = reinterpret_cast<const std::byte*>(serialized_descriptor.data());
  schema.data_len = serialized_descriptor.size();
  planning_info_channel_ =
      make_channel(RawChannel::create("/planning_info", "protobuf", std::move(schema)));

  return true;
}

SceneUpdate Simulator::get_ego_scene_update() const {
  ModelPrimitive ego_model;
  ego_model.url =
      "https://raw.githubusercontent.com/PuYuuu/dive-into-contingency-planning/master/assets/"
      "mesh/lexus.glb";
  ego_model.pose = construct_pose();
  ego_model.scale = {1, 1, 1};
  ego_model.media_type = "model/gltf-binary";
  ego_model.override_color = false;

  SceneEntity ego_car_entity;
  ego_car_entity.id = "ego_car";
  ego_car_entity.frame_id = "base_link";
  ego_car_entity.models.emplace_back(ego_model);
  ego_car_entity.timestamp = TimeUtil::NowTimestamp();
  ego_car_entity.lifetime = Duration{0, 100000000};

  SceneUpdate ego_car_scene_update;
  ego_car_scene_update.entities.emplace_back(ego_car_entity);

  return ego_car_scene_update;
}

SceneUpdate Simulator::get_trajectory_scene_update(void) const {
  SceneUpdate traj_scene_update;
  SceneEntity traj_entity;
  traj_entity.id = "trajectory";
  traj_entity.frame_id = "map";
  traj_entity.timestamp = TimeUtil::NowTimestamp();
  traj_entity.lifetime = Duration{0, 200000000};

  // #0066BFE5
  Color traj_color = Color{0.0, 0.4, 0.75, 0.9};
  {
    std::shared_lock lock(planning_info_mutex_);
    std::vector<Point3> traj_points;
    const auto& optimal_trajectory = planning_info_.optimal_trajectory();
    for (int idx = 0; idx < planning_info_.optimal_trajectory_size(); ++idx) {
      const float x = optimal_trajectory.at(idx).pos_x();
      const float y = optimal_trajectory.at(idx).pos_y();
      Point3 point{x, y, 0.0};
      traj_points.emplace_back(point);
    }
    traj_entity.triangles.emplace_back(create_line_mesh(traj_points, 1.4, traj_color, true));
  }
  traj_scene_update.entities.emplace_back(traj_entity);
  return traj_scene_update;
}

SceneUpdate Simulator::get_lanelets_scene_update(
    const std::vector<std::shared_ptr<Lanelet>>& lanelets) const {
  static SceneUpdate lanelet_scene_update;
  static bool is_hdmap_lanelets_initialized = false;
  if (is_hdmap_lanelets_initialized) {
    return lanelet_scene_update;
  }
  SceneEntity lanelet_entity;
  lanelet_entity.id = "lanelets";
  lanelet_entity.frame_id = "map";
  lanelet_entity.timestamp = TimeUtil::NowTimestamp();
  lanelet_entity.lifetime = Duration{0, 0};

  auto lane_pose = construct_pose();
  // #FFFFFFFF
  auto lane_color = Color{1.0, 1.0, 1.0, 1.0};
  // #E63333FF
  auto road_edge_color = Color{0.9, 0.2, 0.2, 1.0};
  for (const auto& lanelet : lanelets) {
    LOG_INFO(logger_, "Lanelet ID: {}", lanelet->getId());
    LinePrimitive left_border_line;
    left_border_line.type = LinePrimitive::LineType::LINE_STRIP;
    left_border_line.pose = lane_pose;
    left_border_line.thickness = 0.18;
    left_border_line.scale_invariant = false;
    left_border_line.color = lane_color;
    if (lanelet->getAdjacentLeft().adj == nullptr) {
      // left border is road edge
      left_border_line.thickness = 0.25;
      left_border_line.color = road_edge_color;
    }
    std::vector<double> left_xs, left_ys;
    for (const auto& vertex : lanelet->getLeftBorderVertices()) {
      left_xs.push_back(vertex.x);
      left_ys.push_back(vertex.y);
    }
    auto left_spline = CubicSpline2D(left_xs, left_ys);
    for (double s = 0.0; s < left_spline.s.back(); s += 0.5) {
      auto pos = left_spline.calc_position(s);
      Point3 point;
      point.x = pos.x();
      point.y = pos.y();
      point.z = 0.0;
      left_border_line.points.emplace_back(point);
    }
    lanelet_entity.lines.emplace_back(left_border_line);

    LinePrimitive right_border_line;
    right_border_line.type = LinePrimitive::LineType::LINE_STRIP;
    right_border_line.pose = lane_pose;
    right_border_line.thickness = 0.18;
    right_border_line.scale_invariant = false;
    right_border_line.color = lane_color;
    if (lanelet->getAdjacentRight().adj == nullptr) {
      // right border is road edge
      right_border_line.thickness = 0.25;
      right_border_line.color = road_edge_color;
    }
    std::vector<double> right_xs, right_ys;
    for (const auto& vertex : lanelet->getRightBorderVertices()) {
      right_xs.push_back(vertex.x);
      right_ys.push_back(vertex.y);
    }
    auto right_spline = CubicSpline2D(right_xs, right_ys);
    for (double s = 0.0; s < right_spline.s.back(); s += 0.5) {
      auto pos = right_spline.calc_position(s);
      Point3 point;
      point.x = pos.x();
      point.y = pos.y();
      point.z = 0.0;
      right_border_line.points.emplace_back(point);
    }
    lanelet_entity.lines.emplace_back(right_border_line);

    LinePrimitive center_line;
    center_line.type = LinePrimitive::LineType::LINE_LIST;
    center_line.pose = lane_pose;
    center_line.thickness = 0.12;
    center_line.scale_invariant = false;
    center_line.color = lane_color;
    std::vector<double> center_xs, center_ys;
    for (const auto& vertex : lanelet->getCenterVertices()) {
      center_xs.push_back(vertex.x);
      center_ys.push_back(vertex.y);
    }
    auto center_spline = CubicSpline2D(center_xs, center_ys);
    for (double s = 0.0; s < center_spline.s.back(); s += 3.0) {
      auto pos = center_spline.calc_position(s);
      Point3 point;
      point.x = pos.x();
      point.y = pos.y();
      point.z = 0.0;
      center_line.points.emplace_back(point);
    }
    lanelet_entity.lines.emplace_back(center_line);
  }
  lanelet_scene_update.entities.emplace_back(lanelet_entity);
  is_hdmap_lanelets_initialized = true;

  return lanelet_scene_update;
}

void Simulator::update_planning_info(const planning::protos::PlanningInfo& info) {
  std::unique_lock lock(planning_info_mutex_);
  planning_info_.CopyFrom(info);
  planning_info_updated_.store(true);
}

SceneUpdate Simulator::get_sampled_scene_update(void) const {
  SceneUpdate traj_scene_update;
  SceneEntity traj_entity;
  traj_entity.id = "sampled_trajectory";
  traj_entity.frame_id = "map";
  traj_entity.timestamp = TimeUtil::NowTimestamp();
  traj_entity.lifetime = Duration{0, 200000000};

  auto traj_pose = construct_pose();
  // #00CCCCE6
  auto traj_color = Color{0.0, 0.8, 0.8, 0.9};

  {
    std::shared_lock lock(planning_info_mutex_);
    const auto& sampled_trajectory = planning_info_.sample_xs();
    for (int seq_idx = 0; seq_idx < planning_info_.sample_xs_size(); ++seq_idx) {
      LinePrimitive sampled_line;
      sampled_line.type = LinePrimitive::LineType::LINE_STRIP;
      sampled_line.pose = traj_pose;
      sampled_line.thickness = 0.2;
      sampled_line.scale_invariant = false;
      sampled_line.color = traj_color;
      const auto& trajectory = sampled_trajectory.at(seq_idx).trajectory();
      for (int idx = 0; idx < trajectory.size(); ++idx) {
        const float x = trajectory.at(idx).pos_x();
        const float y = trajectory.at(idx).pos_y();
        Point3 point{x, y, 0.0};
        sampled_line.points.emplace_back(point);
      }
      traj_entity.lines.emplace_back(sampled_line);
    }
  }
  traj_scene_update.entities.emplace_back(traj_entity);
  return traj_scene_update;
}

SceneUpdate Simulator::get_reference_line_scene_update(void) const {
  static SceneUpdate ref_line_scene_update;
  static bool is_routing_scene_initialized = false;
  if (is_routing_scene_initialized) {
    return ref_line_scene_update;
  }
  SceneEntity ref_line_entity;
  ref_line_entity.id = "reference_line";
  ref_line_entity.frame_id = "map";
  ref_line_entity.timestamp = TimeUtil::NowTimestamp();
  ref_line_entity.lifetime = Duration{0, 0};

  // #5CD4A8E6
  Color ref_line_color = Color{0.36, 0.83, 0.66, 0.9};
  LinePrimitive ref_line_primitive;
  ref_line_primitive.type = LinePrimitive::LineType::LINE_STRIP;
  ref_line_primitive.pose = construct_pose();
  ref_line_primitive.thickness = 0.6;
  ref_line_primitive.scale_invariant = false;
  ref_line_primitive.color = ref_line_color;
  const double sphere_radius = 0.6 * 2.0;
  Vector3 sphere_size{sphere_radius, sphere_radius, sphere_radius};
  if (routing_reference_line_) {
    for (size_t idx = 0; idx < routing_reference_line_->size(); idx += 5) {
      Point3 point;
      point.x = routing_reference_line_->at(idx).x();
      point.y = routing_reference_line_->at(idx).y();
      point.z = 0.0;
      ref_line_primitive.points.emplace_back(point);
      if (idx % 50 == 0) {
        SpherePrimitive ref_point_sphere;
        ref_point_sphere.pose = construct_pose();
        ref_point_sphere.pose->position->x = point.x;
        ref_point_sphere.pose->position->y = point.y;
        ref_point_sphere.size = sphere_size;
        ref_point_sphere.color = ref_line_color;
        ref_line_entity.spheres.emplace_back(ref_point_sphere);
      }
    }
    ref_line_entity.lines.emplace_back(ref_line_primitive);
    ref_line_scene_update.entities.emplace_back(ref_line_entity);
  }
  is_routing_scene_initialized = true;

  return ref_line_scene_update;
}

std::shared_ptr<ReferenceLine> Simulator::get_reference_line(void) const {
  std::shared_lock lock(reference_line_mutex_);
  if (routing_reference_line_) {
    return routing_reference_line_;
  }

  return nullptr;
}

SceneUpdate Simulator::get_obstacle_list_scene_update(void) const {
  static constexpr double kObstacleHeightVRU = 1.7;
  static constexpr double kObstacleHeightVehicle = 1.6;
  static constexpr double kObstacleHeightBus = 3.0;
  static constexpr Color kObstacleColorVRU = {0.22, 0.43, 0.64, 0.8};      // #3A6EA5
  static constexpr Color kObstacleColorVehicle = {0.83, 0.59, 0.25, 0.8};  // #D4963F
  static constexpr Color kObstacleColorBus = {0.8, 0.2, 0.2, 0.8};         // #CC3333

  SceneUpdate obstacles_scene_update;
  auto obstacles = sim_world_->getObstacles();
  for (const auto& obstacle : obstacles) {
    double obstacle_length = obstacle->getShapePtr()->getLength();
    double obstacle_width = obstacle->getShapePtr()->getWidth();
    double obstacle_heading = obstacle->getCurrentState()->getGlobalOrientation();
    ObstacleType obstacle_type = obstacle->getObstacleType();

    SceneEntity obstacle_entity;
    obstacle_entity.id = "obstacle_" + std::to_string(obstacle->getId());
    obstacle_entity.frame_id = "map";
    obstacle_entity.timestamp = TimeUtil::NowTimestamp();
    obstacle_entity.lifetime = Duration{0, 200000000};

    CubePrimitive cube_marker;
    if (obstacle_type == ObstacleType::pedestrian || obstacle_type == ObstacleType::bicycle ||
        obstacle_type == ObstacleType::motorcycle || obstacle_type == ObstacleType::vru) {
      cube_marker.size = Vector3{obstacle_length, obstacle_width, kObstacleHeightVRU};
      cube_marker.color = kObstacleColorVRU;
    } else if (obstacle_type == ObstacleType::car || obstacle_type == ObstacleType::vehicle ||
               obstacle_type == ObstacleType::parked_vehicle) {
      cube_marker.size = Vector3{obstacle_length, obstacle_width, kObstacleHeightVehicle};
      cube_marker.color = kObstacleColorVehicle;
    } else if (obstacle_type == ObstacleType::bus || obstacle_type == ObstacleType::truck) {
      cube_marker.size = Vector3{obstacle_length, obstacle_width, kObstacleHeightBus};
      cube_marker.color = kObstacleColorBus;
    } else {
      cube_marker.size = Vector3{obstacle_length, obstacle_width, kObstacleHeightVehicle};
      cube_marker.color = kObstacleColorVehicle;
    }
    double half_height = cube_marker.size->z / 2.0;
    Pose obstacle_pose;
    obstacle_pose.position = Vector3{obstacle->getCurrentState()->getXPosition(),
                                     obstacle->getCurrentState()->getYPosition(), half_height};
    obstacle_pose.orientation = yaw_to_quaternion(obstacle_heading);
    cube_marker.pose = obstacle_pose;
    obstacle_entity.cubes.emplace_back(cube_marker);

    TextPrimitive id_text;
    id_text.text = std::to_string(obstacle->getId());
    id_text.pose = cube_marker.pose;
    id_text.color = Color{1.0, 1.0, 1.0, 1.0};
    id_text.font_size = obstacle_width;
    obstacle_entity.texts.emplace_back(id_text);

    ArrowPrimitive heading_arrow;
    heading_arrow.pose = cube_marker.pose;
    heading_arrow.color = cube_marker.color;
    heading_arrow.color->a = 1.0;
    heading_arrow.shaft_length = 1.6;
    heading_arrow.head_length = 0.7;
    heading_arrow.shaft_diameter = 0.3;
    heading_arrow.head_diameter = 0.6;
    double half_length = cube_marker.size->x / 2.0;
    heading_arrow.pose->position->x += (half_length * std::cos(obstacle_heading));
    heading_arrow.pose->position->y += (half_length * std::sin(obstacle_heading));
    obstacle_entity.arrows.emplace_back(heading_arrow);

    obstacles_scene_update.entities.emplace_back(obstacle_entity);
  }

  return obstacles_scene_update;
}

std::shared_ptr<common::ObstacleList> Simulator::get_obstacle_list(void) const {
  auto obstacle_list = std::make_shared<common::ObstacleList>();
  auto obstacles = sim_world_->getObstacles();
  StateInfo current_ego_state = get_ego_state();

  for (const auto& obstacle : obstacles) {
    const double obstacle_x = obstacle->getCurrentState()->getXPosition();
    const double obstacle_y = obstacle->getCurrentState()->getYPosition();
    double distance_to_ego =
        std::hypot(obstacle_x - current_ego_state.x, obstacle_y - current_ego_state.y);
    if (distance_to_ego > 100.0) {
      continue;
    }

    common::Obstacle obs;
    obs.set_id(std::to_string(obstacle->getId()));
    obs.set_type(static_cast<common::ObstacleType>(obstacle->getObstacleType()));
    obs.set_x(obstacle_x);
    obs.set_y(obstacle_y);
    obs.set_length(obstacle->getGeoShape().getLength());
    obs.set_width(obstacle->getGeoShape().getWidth());
    obs.set_heading(obstacle->getCurrentState()->getGlobalOrientation());
    obstacle_list->append(obs);
  }
  return obstacle_list;
}
