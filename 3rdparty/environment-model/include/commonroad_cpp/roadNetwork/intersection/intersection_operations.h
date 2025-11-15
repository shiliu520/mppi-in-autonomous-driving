#pragma once

class World;
class Obstacle;
class Intersection;
class RoadNetwork;
class Lanelet;
class IncomingGroup;
enum class IntersectionType;

namespace intersection_operations {

/**
 * Matches a string to an intersection type
 *
 * @param type for which intersection type should be extracted
 * @return intersection type which corresponds to string or unknown type if string does not match
 */
IntersectionType matchStringToIntersectionType(const std::string &type);

/**
 * Evaluates whether obstacle occupies a incoming lanelet.
 * @param timeStep Time step of interest.
 * @param obs Pointer to obstacle.
 * @param roadNetwork Pointer to road network.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
bool onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs, const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 *
 * @param letK Occupied lanelet of the kth vehicle.
 * @param letP Occupied lanelet of the pth vehicle.
 * @param fov Field of view.
 * @param numIntersections Number of intersections which should be considered for lane generation.
 * @return Boolean indicating whether the two lanelets are part of the same incoming.
 */
bool checkSameIncoming(const std::shared_ptr<Lanelet> &letK, const std::shared_ptr<Lanelet> &letP, double fov,
                       int numIntersections);

/**
 * Calculate left of given incomingGroup
 *
 * @param origin originGroup of interest
 * @param roadNetwork Road network object.
 */
void findLeftOf(const std::shared_ptr<IncomingGroup> &origin, const std::shared_ptr<RoadNetwork> &roadNetwork);

/**
 * Gets the intersection on which the obstacle is currently.
 * @param timeStep Time step of interest.
 * @param world World object
 * @param obstacleK Relevant obstacle.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
std::shared_ptr<Intersection> currentIntersection(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK);

/**
 * Gets the incoming from the intersection on which the obstacle is currently.
 * @param timeStep Time step of interest.
 * @param world World object
 * @param obs Relevant obstacle.
 * @return Boolean indicating whether incoming is occupied by obstacle.
 */
std::shared_ptr<IncomingGroup> currentIncoming(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obs);

} // namespace intersection_operations
