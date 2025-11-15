#pragma once

#include <memory>
#include <vector>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

namespace obstacle_reference {

/**
 * Computes candidate reference lanes for obstacle.
 *
 * @param obstacle Obstacle of interest.
 * @param roadNetwork CommonRoad road network.
 * @param timeStep Time step of interest.
 * @return List of pointers to lanes.
 */
std::vector<std::shared_ptr<Lane>> computeRef(Obstacle &obstacle, const std::shared_ptr<RoadNetwork> &roadNetwork,
                                              size_t timeStep);

/**
 * Computes candidate reference lanes for obstacle given candidate lanes.
 * @param obstacle Obstacle of interest.
 * @param roadNetwork CommonRoad road network.
 * @param timeStep Time step of interest.
 * @param lanes Lane candidates.
 * @return List of pointers to lanes.
 */
std::vector<std::shared_ptr<Lane>> computeRefGivenLanes(Obstacle &obstacle,
                                                        const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        size_t timeStep,
                                                        const std::vector<std::shared_ptr<Lane>> &lanes);

/**
 * Counts occupancies of obstacle on lane over time.
 *
 * @param obstacle Obstacle of interest.
 * @param roadNetwork CommonRoad road network.
 * @param timeStep Time step of interest.
 * @param relevantOccupiedLanes Lane candidates.
 * @return Ordered list of lanes based on number of occupancies.
 */
std::vector<std::shared_ptr<Lane>> countOccupanciesOverTime(Obstacle &obstacle,
                                                            const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                            size_t timeStep,
                                                            std::vector<std::shared_ptr<Lane>> &relevantOccupiedLanes);
}; // namespace obstacle_reference
