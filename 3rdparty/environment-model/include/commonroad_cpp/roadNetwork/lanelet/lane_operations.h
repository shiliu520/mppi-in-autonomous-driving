#pragma once

#include "../road_network.h"
#include "lane.h"

namespace lane_operations {
/**
 * Combines a lanelet and all its successors to lanes. For each successor lanelet a new lane is created. The algorithm
 * stops when the lane reaches a length given by the parameter fov.
 *
 * @param curLanelet Lanelet which is currently at the end of list and for which successors should be added.
 * @param fov Field of view defining length of lane.
 * @param numIntersections Number of intersection which still can be considered for lane creation.
 * @param containedLanelets List of contained lanelets in lane. Required for recursive call.
 * @param offset Offset from the beginning of the reference lanelet.
 * @return List containing a list of lanelets contained in lane.
 */
std::vector<std::vector<std::shared_ptr<Lanelet>>>
combineLaneletAndSuccessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov, int numIntersections,
                                  const std::vector<std::shared_ptr<Lanelet>> &containedLanelets = {},
                                  double offset = 0.0);

std::shared_ptr<Lane> computeLaneFromTwoPoints(const vertex &start, const vertex &end,
                                               const std::shared_ptr<RoadNetwork> &road_network);

/**
 * Combines a lanelet and all its predecessors to lanes. For each predecessor lanelet a new lane is created. The
 * algorithm stops when the lane reaches a length given by the parameter fov.
 *
 * @param curLanelet Lanelet which is currently at the end of list and for which predecessors should be added.
 * @param fov Field of view defining length of lane.
 * @param containedLanelets List of contained lanelets in lane. Required for recursive call.
 * @param numIntersections Number of intersection which still can be considered for lane creation.
 * @param offset Offset from the end of the reference lanelet.
 * @return List containing a list of lanelets contained in lane.
 */
std::vector<std::vector<std::shared_ptr<Lanelet>>>
combineLaneletAndPredecessorsToLane(const std::shared_ptr<Lanelet> &curLanelet, double fov, int numIntersections,
                                    std::vector<std::shared_ptr<Lanelet>> containedLanelets = {}, double offset = 0.0);

/**
 * Creates lanes which are originating from given set of lanelets.
 *
 * @param initialLanelets Initial lanelets based on which lanes should be created.
 * @param roadNetwork Pointer to road network.
 * @param fovRear Field of view behind obstacle which defines length of lanes.
 * @param fovFront Field of view in front of obstacle which defines length of lanes.
 * @param numIntersections Number of intersection which still can be considered for lane creation.
 * @param position Position which should be used as offset.
 * @return List of pointers to lanes originating from given lanelets.
 */
std::vector<std::shared_ptr<Lane>>
createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                            const std::shared_ptr<RoadNetwork> &roadNetwork, double fovRear, double fovFront,
                            int numIntersections, vertex position);

/**
 * Creates lane objects given set of lanelets which form lane.
 *
 * @param containedLanelets List of pointers to lanelets.
 * @param newId ID of new lane.
 * @return Pointer to new lane.
 */
std::shared_ptr<Lane> createLaneByContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets,
                                                    size_t newId);

/**
 * Evaluates whether occupied lanelets are part of two directly adjacent lanes.
 *
 * @param laneOne Pointer to first lane.
 * @param laneTwo Pointer to second lane.
 * @param relevantLanelets Lanelets which should be evaluated, e.g., occupied lanelets. If only a single lane is
 * provided the algorithm returns false.
 * @return Boolean indicating whether lanelets part of two adjacent lanes.
 */
bool areLaneletsInDirectlyAdjacentLanes(const std::shared_ptr<Lane> &laneOne, const std::shared_ptr<Lane> &laneTwo,
                                        const std::vector<std::shared_ptr<Lanelet>> &relevantLanelets);

/**
 * Extracts list of lanelets part of give list of lanes.
 *
 * @param lanes List of lanes.
 * @return List of lanelets.
 */
std::vector<std::shared_ptr<Lanelet>> extractLaneletsFromLanes(const std::vector<std::shared_ptr<Lane>> &lanes);

/**
 * Combines lanelets from several lanelets list to one list.
 *
 * @param lanes List of lists containing lanelets.
 * @return List of lanelets.
 */
std::vector<std::shared_ptr<Lanelet>>
combineLaneLanelets(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &lanes);

} // namespace lane_operations
