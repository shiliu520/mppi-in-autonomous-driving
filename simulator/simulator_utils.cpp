#include "simulator_utils.hpp"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"

std::tuple<foxglove::schemas::Vector3, foxglove::schemas::Color> get_obstacle_size_and_color(
    ObstacleType obstacle_type, double obstacle_length, double obstacle_width) {
  foxglove::schemas::Vector3 size;
  foxglove::schemas::Color color;
  if (obstacle_type == ObstacleType::pedestrian || obstacle_type == ObstacleType::bicycle ||
      obstacle_type == ObstacleType::motorcycle || obstacle_type == ObstacleType::vru) {
    size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightVRU};
    color = kObstacleColorVRU;
  } else if (obstacle_type == ObstacleType::car || obstacle_type == ObstacleType::vehicle ||
             obstacle_type == ObstacleType::parked_vehicle) {
    size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightVehicle};
    color = kObstacleColorVehicle;
  } else if (obstacle_type == ObstacleType::bus || obstacle_type == ObstacleType::truck) {
    size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightBus};
    color = kObstacleColorBus;
  } else {
    size = foxglove::schemas::Vector3{obstacle_length, obstacle_width, kObstacleHeightVehicle};
    color = kObstacleColorVehicle;
  }
  return {size, color};
}

foxglove::schemas::TriangleListPrimitive create_line_mesh(
    const std::vector<foxglove::schemas::Point3>& points, double thickness,
    const foxglove::schemas::Color& color, bool gradient_fade /* = false */) {
  foxglove::schemas::TriangleListPrimitive mesh;

  if (points.size() < 2) {
    return mesh;  // Need at least 2 points to draw a line
  }

  const double half_thickness = thickness * 0.5;
  std::vector<foxglove::schemas::Point3> left_vertices;
  std::vector<foxglove::schemas::Point3> right_vertices;
  std::vector<foxglove::schemas::Color> vertex_colors;

  // Generate vertices along the line with perpendicular offset
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& curr = points[i];
    foxglove::schemas::Vector3 direction{0.0, 0.0, 0.0};

    // Calculate tangent direction
    if (i == 0) {
      // First point: use direction to next point
      direction.x = points[i + 1].x - curr.x;
      direction.y = points[i + 1].y - curr.y;
      direction.z = points[i + 1].z - curr.z;
    } else if (i == points.size() - 1) {
      // Last point: use direction from previous point
      direction.x = curr.x - points[i - 1].x;
      direction.y = curr.y - points[i - 1].y;
      direction.z = curr.z - points[i - 1].z;
    } else {
      // Middle points: use average of incoming and outgoing directions
      double dx1 = curr.x - points[i - 1].x;
      double dy1 = curr.y - points[i - 1].y;
      double dz1 = curr.z - points[i - 1].z;
      double dx2 = points[i + 1].x - curr.x;
      double dy2 = points[i + 1].y - curr.y;
      double dz2 = points[i + 1].z - curr.z;
      direction.x = dx1 + dx2;
      direction.y = dy1 + dy2;
      direction.z = dz1 + dz2;
    }

    // Normalize direction
    double length = std::sqrt(direction.x * direction.x + direction.y * direction.y +
                              direction.z * direction.z);
    if (length > 1e-10) {
      direction.x /= length;
      direction.y /= length;
      direction.z /= length;
    }

    // Calculate perpendicular offset (assuming line is in XY plane, perpendicular is in XY)
    foxglove::schemas::Vector3 perpendicular;
    perpendicular.x = -direction.y;
    perpendicular.y = direction.x;
    perpendicular.z = 0.0;

    // Create left and right vertices
    foxglove::schemas::Point3 left_vertex;
    left_vertex.x = curr.x + perpendicular.x * half_thickness;
    left_vertex.y = curr.y + perpendicular.y * half_thickness;
    left_vertex.z = curr.z;

    foxglove::schemas::Point3 right_vertex;
    right_vertex.x = curr.x - perpendicular.x * half_thickness;
    right_vertex.y = curr.y - perpendicular.y * half_thickness;
    right_vertex.z = curr.z;

    left_vertices.push_back(left_vertex);
    right_vertices.push_back(right_vertex);

    // Calculate color with gradient fade if enabled
    if (gradient_fade) {
      foxglove::schemas::Color vertex_color = color;
      // Non-linear fade: use quadratic function (t^2) for slow start, fast end
      double t = static_cast<double>(i) / static_cast<double>(points.size() - 1);
      double alpha_multiplier = 1.0 - (t * t);  // Quadratic decay
      vertex_color.a = color.a * alpha_multiplier;
      vertex_colors.push_back(vertex_color);
    }
  }

  // Create triangles connecting adjacent vertex pairs
  for (size_t i = 0; i < points.size() - 1; ++i) {
    // First triangle: left[i], right[i], left[i+1]
    mesh.points.push_back(left_vertices[i]);
    mesh.points.push_back(right_vertices[i]);
    mesh.points.push_back(left_vertices[i + 1]);

    // Second triangle: right[i], right[i+1], left[i+1]
    mesh.points.push_back(right_vertices[i]);
    mesh.points.push_back(right_vertices[i + 1]);
    mesh.points.push_back(left_vertices[i + 1]);

    // Add per-vertex colors if gradient fade is enabled
    if (gradient_fade) {
      // First triangle vertices
      mesh.colors.push_back(vertex_colors[i]);
      mesh.colors.push_back(vertex_colors[i]);
      mesh.colors.push_back(vertex_colors[i + 1]);

      // Second triangle vertices
      mesh.colors.push_back(vertex_colors[i]);
      mesh.colors.push_back(vertex_colors[i + 1]);
      mesh.colors.push_back(vertex_colors[i + 1]);
    }
  }

  // Set uniform color for the entire mesh if no gradient
  if (!gradient_fade) {
    mesh.color = color;
  }

  return mesh;
}

std::pair<bool, bool> is_lanelet_at_road_edge(
    const std::shared_ptr<Lanelet>& lanelet,
    const std::shared_ptr<RoadNetwork>& road_network) {
  
  if (!lanelet || !road_network) {
    return {false, false};
  }

  bool is_leftmost = false;
  bool is_rightmost = false;

  // Check left side
  const auto& left_adjacent = lanelet->getAdjacentLeft();
  if (left_adjacent.adj == nullptr) {
    // No adjacent lanelet on the left - need further check for merge/diverge scenarios
    is_leftmost = true;
    
    // Check if this is a merge/diverge scenario
    const auto& successors = lanelet->getSuccessors();
    if (!successors.empty()) {
      // Check if any successor has other predecessors (indicating merge)
      for (const auto& successor : successors) {
        if (!successor) continue;
        
        const auto& succ_predecessors = successor->getPredecessors();
        if (succ_predecessors.size() > 1) {
          // Multiple lanes merge - need to check relative position
          // Get the first point of current lanelet
          const auto& curr_center = lanelet->getCenterVertices();
          if (curr_center.empty()) continue;
          
          double curr_y = curr_center.front().y;
          double curr_x = curr_center.front().x;
          
          // Check if there's any other predecessor on the left (larger y in most cases)
          bool has_left_neighbor = false;
          for (const auto& pred : succ_predecessors) {
            if (!pred || pred->getId() == lanelet->getId()) continue;
            
            const auto& pred_center = pred->getCenterVertices();
            if (pred_center.empty()) continue;
            
            double pred_y = pred_center.front().y;
            double pred_x = pred_center.front().x;
            
            // Calculate the relative position using cross product
            // Vector from current to predecessor
            double dx = pred_x - curr_x;
            double dy = pred_y - curr_y;
            
            // Direction of current lanelet (use first two points)
            if (curr_center.size() < 2) continue;
            double dir_x = curr_center[1].x - curr_center[0].x;
            double dir_y = curr_center[1].y - curr_center[0].y;
            
            // Cross product: if positive, pred is on the left
            double cross = dir_x * dy - dir_y * dx;
            if (cross > 0.1) {  // Small threshold to avoid numerical errors
              has_left_neighbor = true;
              break;
            }
          }
          
          if (has_left_neighbor) {
            is_leftmost = false;
          }
          break;
        }
      }
    }
    
  } else if (left_adjacent.oppositeDir) {
    // Adjacent lanelet exists but in opposite direction - this is a road edge
    is_leftmost = true;
  } else {
    // Check line marking - road edges typically have curb or solid markings
    LineMarking left_marking = lanelet->getLineMarkingLeft();
    if (left_marking == LineMarking::curb || 
        left_marking == LineMarking::lowered_curb) {
      is_leftmost = true;
    }
  }

  // Check right side
  const auto& right_adjacent = lanelet->getAdjacentRight();
  if (right_adjacent.adj == nullptr) {
    // No adjacent lanelet on the right - need further check for merge/diverge scenarios
    is_rightmost = true;
    
    // Check if this is a merge/diverge scenario
    const auto& successors = lanelet->getSuccessors();
    if (!successors.empty()) {
      // Check if any successor has other predecessors (indicating merge)
      for (const auto& successor : successors) {
        if (!successor) continue;
        
        const auto& succ_predecessors = successor->getPredecessors();
        if (succ_predecessors.size() > 1) {
          // Multiple lanes merge - need to check relative position
          // Get the first point of current lanelet
          const auto& curr_center = lanelet->getCenterVertices();
          if (curr_center.empty()) continue;
          
          double curr_y = curr_center.front().y;
          double curr_x = curr_center.front().x;
          
          // Check if there's any other predecessor on the right (smaller y in most cases)
          bool has_right_neighbor = false;
          for (const auto& pred : succ_predecessors) {
            if (!pred || pred->getId() == lanelet->getId()) continue;
            
            const auto& pred_center = pred->getCenterVertices();
            if (pred_center.empty()) continue;
            
            double pred_y = pred_center.front().y;
            double pred_x = pred_center.front().x;
            
            // Calculate the relative position using cross product
            // Vector from current to predecessor
            double dx = pred_x - curr_x;
            double dy = pred_y - curr_y;
            
            // Direction of current lanelet (use first two points)
            if (curr_center.size() < 2) continue;
            double dir_x = curr_center[1].x - curr_center[0].x;
            double dir_y = curr_center[1].y - curr_center[0].y;
            
            // Cross product: if negative, pred is on the right
            double cross = dir_x * dy - dir_y * dx;
            if (cross < -0.1) {  // Small threshold to avoid numerical errors
              has_right_neighbor = true;
              break;
            }
          }
          
          if (has_right_neighbor) {
            is_rightmost = false;
          }
          break;
        }
      }
    }
    
  } else if (right_adjacent.oppositeDir) {
    // Adjacent lanelet exists but in opposite direction - this is a road edge
    is_rightmost = true;
  } else {
    // Check line marking - road edges typically have curb or solid markings
    LineMarking right_marking = lanelet->getLineMarkingRight();
    if (right_marking == LineMarking::curb || 
        right_marking == LineMarking::lowered_curb) {
      is_rightmost = true;
    }
  }

  return {is_leftmost, is_rightmost};
}

std::pair<bool, bool> should_draw_lanelet_borders(
    const std::shared_ptr<Lanelet>& lanelet,
    const std::shared_ptr<RoadNetwork>& road_network) {
  
  if (!lanelet || !road_network) {
    return {true, true};
  }

  bool draw_left = false;
  bool draw_right = false;

  // Get edge detection results
  auto [is_leftmost, is_rightmost] = is_lanelet_at_road_edge(lanelet, road_network);
  
  // Check if this is in a merge/diverge scenario
  bool is_in_merge_scenario = false;
  const auto& successors = lanelet->getSuccessors();
  
  for (const auto& successor : successors) {
    if (!successor) continue;
    const auto& succ_predecessors = successor->getPredecessors();
    if (succ_predecessors.size() > 1) {
      is_in_merge_scenario = true;
      break;
    }
  }

  if (is_in_merge_scenario) {
    // In merge scenario: only draw outer borders
    // Leftmost lane draws left border, rightmost draws right border
    draw_left = is_leftmost;
    draw_right = is_rightmost;
  } else {
    // Normal scenario: check adjacency
    const auto& left_adjacent = lanelet->getAdjacentLeft();
    const auto& right_adjacent = lanelet->getAdjacentRight();
    
    // Draw left border if:
    // 1. No left adjacent lane (road edge), OR
    // 2. Left adjacent is opposite direction (center line)
    if (left_adjacent.adj == nullptr || left_adjacent.oppositeDir) {
      draw_left = true;
    } else {
      // Check line marking - draw if it's a special marking
      LineMarking left_marking = lanelet->getLineMarkingLeft();
      if (left_marking == LineMarking::solid || 
          left_marking == LineMarking::dashed ||
          left_marking == LineMarking::curb || 
          left_marking == LineMarking::lowered_curb) {
        draw_left = true;
      }
    }
    
    // Draw right border if:
    // 1. No right adjacent lane (road edge), OR
    // 2. Right adjacent is opposite direction (center line), OR
    // 3. This is to ensure at least one border between adjacent same-direction lanes
    if (right_adjacent.adj == nullptr || right_adjacent.oppositeDir) {
      draw_right = true;
    } else {
      // For same-direction adjacent lanes, draw right border to avoid missing lines
      // (the adjacent lane on the right will not draw its left border)
      LineMarking right_marking = lanelet->getLineMarkingRight();
      if (right_marking == LineMarking::solid || 
          right_marking == LineMarking::dashed ||
          right_marking == LineMarking::curb || 
          right_marking == LineMarking::lowered_curb) {
        draw_right = true;
      }
    }
    
    // Always draw at least left border if it's at road edge
    if (is_leftmost) {
      draw_left = true;
    }
    if (is_rightmost) {
      draw_right = true;
    }
  }

  return {draw_left, draw_right};
}
