#pragma once

#include <memory>
#include <nanobind/nanobind.h>
#include <vector>

namespace nb = nanobind;

class WorldParameters;
class Lanelet;
class Obstacle;
class State;
class Occupancy;
class TrafficSign;
class TrafficLight;
class Intersection;
class StopLine;
class TrafficLightCycleElement;

namespace TranslatePythonTypes {
/**
 * Converts Python lanelet objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights
 * @return List of pointers to lanelet objects.
 */
std::vector<std::shared_ptr<Lanelet>> convertLanelets(const nb::handle &py_laneletNetwork,
                                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights);

/**
 * Converts Python dynamic obstacle objects to C++ representation.
 *
 * @param py_obstacles List of Python version of dynamic obstacles.
 * @param worldParams World parameters.
 * @param ego Flag to indicate if the obstacles are ego vehicles.
 * @return List of pointers to obstacle objects.
 */
std::vector<std::shared_ptr<Obstacle>> convertObstacles(const nb::list &py_obstacles,
                                                        const WorldParameters &worldParams, bool ego);

/**
 * Converts Python State.
 *
 * @param py_state Python state object.
 * @return C++ State.
 */
std::shared_ptr<State> extractState(nb::handle py_state);

/**
* Converts Python Occupancy.
 @param py_occupancy Python occupancy object.
    @return C++ Occupancy.
*/
std::shared_ptr<Occupancy> extractOccupancy(nb::handle py_occupancy);

/**
 * Create dynamic obstacle.
 * @param py_singleObstacle Python dynamic obstacle.
 * @return C++ dynamic obstacle.
 */
std::shared_ptr<Obstacle> createDynamicObstacle(nb::handle py_singleObstacle);

/**
 * Create static obstacle.
 * @param py_singleObstacle Python static obstacle.
 * @return C++ static obstacle.
 */
std::shared_ptr<Obstacle> createStaticObstacle(nb::handle py_singleObstacle);

/**
 * Converts Python traffic sign objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @return List of pointers to traffic sign objects.
 */
std::vector<std::shared_ptr<TrafficSign>> convertTrafficSigns(const nb::handle &py_laneletNetwork);

/**
 * Converts Python traffic light objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @return List of pointers to traffic light objects.
 */
std::vector<std::shared_ptr<TrafficLight>> convertTrafficLights(const nb::handle &py_laneletNetwork);

std::vector<TrafficLightCycleElement> convertTrafficLightCycleElements(const nb::handle &py_trafficLightCycle);

/**
 * Converts Python intersection objects to C++ representation.
 *
 * @param py_laneletNetwork Python lanelet network object.
 * @param lanelets List of pointers to lanelets.
 * @return List of pointers to intersection objects.
 */
std::vector<std::shared_ptr<Intersection>> convertIntersections(const nb::handle &py_laneletNetwork,
                                                                const std::vector<std::shared_ptr<Lanelet>> &lanelets);

/**
 * Converts Python stop line object to C++ representation.
 *
 * @param py_stopLine Python stop line object.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights
 * @return
 */
std::shared_ptr<StopLine> convertStopLine(const nb::handle &py_stopLine,
                                          const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                          const std::vector<std::shared_ptr<TrafficLight>> &trafficLights);

} // namespace TranslatePythonTypes
