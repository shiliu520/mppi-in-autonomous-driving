#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "simulator_utils.hpp"

#include <vector>
#include <cmath>

std::pair<double, double> compute_road_edge_distances(
    double px, double py, const std::shared_ptr<RoadNetwork>& road_network) {
  const double kDefaultDistance = 20.0;

  if (!road_network) {
    return {kDefaultDistance, kDefaultDistance};
  }

  // Find lanelet containing this point
  auto lanelets = road_network->findLaneletsByPosition(px, py);
  if (lanelets.empty()) {
    return {kDefaultDistance, kDefaultDistance};
  }

  auto lanelet = lanelets[0];
  double left_dist = kDefaultDistance;
  double right_dist = kDefaultDistance;
  bool is_in_intersection = lanelet->getLaneletTypes().count(LaneletType::intersection) > 0;

  if (is_in_intersection) {
    return {2.0, 2.0};  // Approximate value for intersections
  }

  // Helper lambda to compute distance to polyline
  auto computeDistanceToPolyline = [](double px, double py, const std::vector<vertex>& vertices) {
    if (vertices.empty()) {
      return 20.0;
    }

    double min_distance = 20.0;
    for (size_t i = 0; i < vertices.size() - 1; ++i) {
      const double x1 = vertices[i].x;
      const double y1 = vertices[i].y;
      const double x2 = vertices[i + 1].x;
      const double y2 = vertices[i + 1].y;

      const double dx = px - x1;
      const double dy = py - y1;
      const double seg_dx = x2 - x1;
      const double seg_dy = y2 - y1;
      const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

      if (seg_len_sq < 1e-10) {
        const double dist = std::hypot(dx, dy);
        min_distance = std::min(min_distance, dist);
        continue;
      }

      double t = (dx * seg_dx + dy * seg_dy) / seg_len_sq;
      t = std::max(0.0, std::min(1.0, t));

      const double closest_x = x1 + t * seg_dx;
      const double closest_y = y1 + t * seg_dy;
      const double dist = std::hypot(px - closest_x, py - closest_y);
      min_distance = std::min(min_distance, dist);
    }
    return min_distance;
  };

  // Compute distance to right road edge
  auto rightmost_lanelet = lanelet;
  // Find rightmost lanelet considering merge/diverge scenarios
  while (!rightmost_lanelet->getAdjacentRight().oppositeDir &&
         rightmost_lanelet->getAdjacentRight().adj != nullptr) {
    rightmost_lanelet = rightmost_lanelet->getAdjacentRight().adj;
  }

  // Check if there's a merge scenario that might have a lane further right
  const auto& successors = rightmost_lanelet->getSuccessors();
  for (const auto& successor : successors) {
    if (!successor) continue;
    const auto& succ_predecessors = successor->getPredecessors();
    if (succ_predecessors.size() > 1) {
      // Multiple lanes merge - find the actual rightmost
      const auto& curr_center = rightmost_lanelet->getCenterVertices();
      if (!curr_center.empty() && curr_center.size() >= 2) {
        double curr_y = curr_center.front().y;
        double curr_x = curr_center.front().x;
        double dir_x = curr_center[1].x - curr_center[0].x;
        double dir_y = curr_center[1].y - curr_center[0].y;

        for (const auto& pred : succ_predecessors) {
          if (!pred || pred->getId() == rightmost_lanelet->getId()) continue;
          const auto& pred_center = pred->getCenterVertices();
          if (pred_center.empty()) continue;

          double pred_y = pred_center.front().y;
          double pred_x = pred_center.front().x;
          double dx = pred_x - curr_x;
          double dy = pred_y - curr_y;
          double cross = dir_x * dy - dir_y * dx;

          // If cross product is negative, pred is on the right
          if (cross < -0.1) {
            rightmost_lanelet = pred;
            break;
          }
        }
      }
      break;
    }
  }

  const auto& right_vertices = rightmost_lanelet->getRightBorderVertices();
  right_dist = computeDistanceToPolyline(px, py, right_vertices);

  // Compute distance to left road edge
  auto leftmost_lanelet = lanelet;
  // Find leftmost lanelet considering merge/diverge scenarios
  while (!leftmost_lanelet->getAdjacentLeft().oppositeDir &&
         leftmost_lanelet->getAdjacentLeft().adj != nullptr) {
    leftmost_lanelet = leftmost_lanelet->getAdjacentLeft().adj;
  }

  // Check if there's a merge scenario that might have a lane further left
  const auto& left_successors = leftmost_lanelet->getSuccessors();
  for (const auto& successor : left_successors) {
    if (!successor) continue;
    const auto& succ_predecessors = successor->getPredecessors();
    if (succ_predecessors.size() > 1) {
      // Multiple lanes merge - find the actual leftmost
      const auto& curr_center = leftmost_lanelet->getCenterVertices();
      if (!curr_center.empty() && curr_center.size() >= 2) {
        double curr_y = curr_center.front().y;
        double curr_x = curr_center.front().x;
        double dir_x = curr_center[1].x - curr_center[0].x;
        double dir_y = curr_center[1].y - curr_center[0].y;

        for (const auto& pred : succ_predecessors) {
          if (!pred || pred->getId() == leftmost_lanelet->getId()) continue;
          const auto& pred_center = pred->getCenterVertices();
          if (pred_center.empty()) continue;

          double pred_y = pred_center.front().y;
          double pred_x = pred_center.front().x;
          double dx = pred_x - curr_x;
          double dy = pred_y - curr_y;
          double cross = dir_x * dy - dir_y * dx;

          // If cross product is positive, pred is on the left
          if (cross > 0.1) {
            leftmost_lanelet = pred;
            break;
          }
        }
      }
      break;
    }
  }

  const auto& left_vertices = leftmost_lanelet->getLeftBorderVertices();
  left_dist = computeDistanceToPolyline(px, py, left_vertices);

  return {left_dist, right_dist};
}
