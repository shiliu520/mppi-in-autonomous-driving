#pragma once

#include <memory>
#include <set>
#include <vector>
class Obstacle;
class Intersection;
class RoadNetwork;
class Lanelet;
struct vertex;
enum class ObstacleType;
enum class Direction;

namespace obstacle_operations {

/**
 * Returns the obstacle which corresponds to a given obstacle ID.
 *
 * @param obstacleId obstacle ID
 * @return pointer to obstacle
 */
std::shared_ptr<Obstacle> getObstacleById(const std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                          size_t obstacleId);

/**
 * Checks if a line is in front
 *
 * @param line Line of interest
 * @param obs Obstacle of interest
 * @param timeStep timestep of interest
 * @param roadNetwork Roadnetwork
 * @return bool
 */
bool lineInFrontOfObstacle(const std::pair<vertex, vertex> &line, const std::shared_ptr<Obstacle> &obs, size_t timeStep,
                           const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Matches a string to an obstacle type
 *
 * @param string for which obstacle type should be extracted
 * @return obstacle type which corresponds to string or unknown type if string does not match
 */
ObstacleType matchStringToObstacleType(const std::string &type);

/**
 * Calculates the min distance from the shape to a line
 *
 * @param line Line of interest
 * @param shape vehicle shape
 * @return double
 */
double minDistanceToPoint(size_t timeStep, const std::pair<vertex, vertex> &line,
                          const std::shared_ptr<Obstacle> &obstacleK);

/**
 * Computes obstacle which is directly left of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which directly left obstacle is computed.
 * @param roadNetwork Relevant road network
 * @return Obstacle directly left.
 */
std::shared_ptr<Obstacle> obstacleDirectlyLeft(size_t timeStep, const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes all obstacles which are left of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which left obstacles are computed.
 * @return List of obstacles located left of given obstacle.
 */
std::vector<std::shared_ptr<Obstacle>> obstaclesLeft(size_t timeStep,
                                                     const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                     const std::shared_ptr<Obstacle> &obstacleK);

/**
 * Computes all obstacle which are left or right adjacent to a given obstacle k based on kth reference path.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which adjacent obstacles are computed.
 * @return List of adjacent obstacles.
 */
std::vector<std::shared_ptr<Obstacle>> obstaclesAdjacent(size_t timeStep,
                                                         const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                         const std::shared_ptr<Obstacle> &obstacleK);

/**
 * Computes obstacle which is directly right of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which directly right obstacle is computed.
 * @param roadNetwork Relevant road network
 * @return Obstacle directly right.
 */
std::shared_ptr<Obstacle> obstacleDirectlyRight(size_t timeStep,
                                                const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes all obstacles which are right of a given obstacle.
 *
 * @param timeStep Time step of interest.
 * @param obstacles List of relevant obstacles.
 * @param obstacleK Obstacle based on which right obstacles are computed.
 * @param roadNetwork Relevant road network
 * @return List of obstacles located right of given obstacle.
 */
std::vector<std::shared_ptr<Obstacle>> obstaclesRight(size_t timeStep,
                                                      const std::vector<std::shared_ptr<Obstacle>> &obstacles,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Computes the set of lanelets on the right side of the obstacle not including occupied lanelets.
 *
 * @param timeStep Time step of interest.
 * @param obs Obstacle based on which right lanelets are computed.
 * @param roadNetwork Relevant road network
 * @return Set of lanelets on the right side of the obstacle.
 */
std::set<std::shared_ptr<Lanelet>> laneletsRightOfObstacle(size_t timeStep,
                                                           const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                           const std::shared_ptr<Obstacle> &obs);

/**
 * Computes the set of lanelets on the left side of the obstacle not including occupied lanelets.
 *
 * @param timeStep Time step of interest.
 * @param obs Obstacle based on which left lanelets are computed.
 * @param roadNetwork Relevant road network
 * @return Set of lanelets on the left side of the obstacle.
 */
std::set<std::shared_ptr<Lanelet>> laneletsLeftOfObstacle(size_t timeStep,
                                                          const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                          const std::shared_ptr<Obstacle> &obs);

/**
 * Computes the set of lanelets parallel to the obstacle given side.
 * @param timeStep Time step of interest.
 * @param roadNetwork Relevant road network
 * @param obs Obstacle based on which left lanelets are computed.
 * @param side Side of interest.
 * @return Set of lanelets on the side of interest of the obstacle.
 */
std::set<std::shared_ptr<Lanelet>> laneletsParallelToObstacle(size_t timeStep,
                                                              const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                              const std::shared_ptr<Obstacle> &obs, Direction side);

/**
 * Returns intersections of the occupied lanes for an obstacle at a specific time step.
 *
 * @param timeStep Time step of interest.
 * @param roadNetwork Road network object.
 * @param obs Obstacle of interest.
 * @return List of pointers to intersections.
 */
std::vector<std::shared_ptr<Intersection>> getIntersections(size_t timeStep,
                                                            const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                            const std::shared_ptr<Obstacle> &obs);

/**
 * Computes the distance of an obstacle to the end of the given lane.
 *
 * @param xPosition X-coordinate-point of interest.
 * @param yPosition Y-coordinate-point of interest.
 * @param roadNetwork Road network.
 * @param obs Obstacle of Interest
 * @param timeStep Time step of interest.
 * @return Distance to the end of the given lane.
 */
double drivingDistanceToCoordinatePoint(double xPosition, double yPosition,
                                        const std::shared_ptr<RoadNetwork> &roadNetwork,
                                        const std::shared_ptr<Obstacle> &obs, size_t timeStep);

} // namespace obstacle_operations
