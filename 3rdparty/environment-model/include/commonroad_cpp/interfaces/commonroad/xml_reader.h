#pragma once

#include <memory>
#include <string>
#include <vector>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/obstacle/signal_state.h"
#include "commonroad_cpp/planning_problem.h"

class Intersection;
class Obstacle;
class State;
class World;
class Lanelet;
class TrafficSign;
class TrafficLight;
class Occupancy;

namespace pugi {
class xml_node;
}

namespace XMLReader {
/**
 * Function for extracting time step size.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return Time step size of scenario [s].
 */
double extractTimeStepSize(const std::string &xmlFile);

/**
 * Function for creating obstacles.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created obstacles.
 */
std::vector<std::shared_ptr<Obstacle>> createObstacleFromXML(const std::string &xmlFile);

/**
 * Function for creating lanelets.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @param trafficSigns List of pointers to traffic signs.
 * @param trafficLights List of pointers to traffic lights.
 * @return List of pointers to created lanelets.
 */
std::vector<std::shared_ptr<Lanelet>>
createLaneletFromXML(const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> trafficSigns = {},
                     std::vector<std::shared_ptr<TrafficLight>> trafficLights = {});

/*
 * Convenient function for creating a world from XML. Note: EgoVehicle is not initialized.
 * @param xmlFile Loaded CommonRoad XML file.
 */
std::shared_ptr<World> createWorldFromXML(const std::string &xmlFile);

/**
 * Function for creating traffic signs.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created traffic signs.
 */
std::vector<std::shared_ptr<TrafficSign>> createTrafficSignFromXML(const std::string &xmlFile);

/**
 * Extract country from XML benchmark ID.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created traffic signs.
 */
SupportedTrafficSignCountry extractCountryFromXML(const std::string &xmlFile);

/**
 * Function for creating traffic lights.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created traffic lights.
 */
std::vector<std::shared_ptr<TrafficLight>> createTrafficLightFromXML(const std::string &xmlFile);

/**
 * Function for creating intersections.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @param lanelets List of pointers to lanelets.
 * @return List of pointers to created intersections.
 */
std::vector<std::shared_ptr<Intersection>>
createIntersectionFromXML(const std::string &xmlFile, const std::vector<std::shared_ptr<Lanelet>> &lanelets);

/**
 * Function for creating planning problems.
 *
 * @param xmlFile Loaded CommonRoad XML file.
 * @return List of pointers to created planning problems.
 */
std::vector<std::shared_ptr<PlanningProblem>> createPlanningProblemFromXML(const std::string &xmlFile);

/**
 * Extracts an initial state from a XML node element.
 *
 * @param child XML node element.
 * @return Pointer to initial state.
 */
std::shared_ptr<State> extractInitialState(const pugi::xml_node &child);

/**
 * Extracts an signal state from a XML node element.
 *
 * @param child XML node element.
 * @return Pointer to signal state.
 */
std::shared_ptr<SignalState> extractSignalState(const pugi::xml_node &child);

/**
 * Extracts a state from an XML node element.
 *
 * @param states XML node element.
 * @return Pointer to state.
 */
std::shared_ptr<State> extractState(const pugi::xml_node &states);

std::shared_ptr<Occupancy> extractOccupancy(const pugi::xml_node &child);

/**
 * Extracts shape from an XML node and assigns it to obstacle.
 *
 * @param obstacle Obstacle for which state should be set.
 * @param child XML node element
 */
void extractObstacleShape(const std::shared_ptr<Obstacle> &obstacle, pugi::xml_node child);

/**
 * Creates a dynamic obstacle from a XML node element and adds the obstacle to provided vector.
 *
 * @param obstacleList Vector in which new dynamic obstacle are stored.
 * @param roadElements XML node element.
 */
void createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList, const pugi::xml_node &roadElements);

/**
 * Creates a static obstacle from a XML node element and adds the obstacle to provided vector.
 *
 * @param obstacleList Vector in which new static obstacle are stored.
 * @param roadElements XML node element.
 */
void extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList, const pugi::xml_node &roadElements);

/**
 * Initializes all lanelets without information using default constructor.
 *
 * @param tempLaneletContainer Vector in which new lanelets are stored.
 * @param commonRoad XML node element.
 * @return Number of created lanelets.
 */
size_t initializeLanelets(std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                          const pugi::xml_node &commonRoad);

/**
 * Extracts lanelet boundary for a lanelet.
 *
 * @param tempLaneletContainer Container with references to all lanelets.
 * @param arrayIndex Index of lanelet within container for which boundary should be extracted.
 * @param child XML node element.
 * @param side String which defines whether left or right lanelet boundary should be extracted.
 */
void extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, size_t arrayIndex,
                            const pugi::xml_node &child, const char *side);

/**
 * Extracts successor or predecessor lanelet for a lanelet.
 *
 * @param tempLaneletContainer Container with references to all lanelets.
 * @param arrayIndex Index of lanelet within container for which successor/predecessor should be extracted.
 * @param child XML node element.
 * @param type String which defines whether successor or predecessor lanelet should be extracted.
 */
void extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, size_t arrayIndex,
                          const pugi::xml_node &child, const char *type);

/**
 * Extracts left or right adjacent lanelet for a lanelet.
 *
 * @param tempLaneletContainer Container with references to all lanelets.
 * @param arrayIndex Index of lanelet within container for which adjacent lanelet should be extracted.
 * @param child XML node element.
 * @param type String which defines whether left or right adjacent lanelet should be extracted.
 */
void extractLaneletAdjacency(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer, size_t arrayIndex,
                             const pugi::xml_node &child, const char *type);

void extractPlanningProblem(std::vector<std::shared_ptr<PlanningProblem>> &planningProblemList,
                            const pugi::xml_node &roadElements);
} // namespace XMLReader
