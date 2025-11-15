#pragma once

#include <commonroad_cpp/geometry/rectangle.h>
#include <fstream>
#include <memory>
#include <tuple>
#include <variant>
#include <vector>

#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/roadNetwork/lanelet/bound.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.h"
#include "commonroad_cpp/scenario.h"
#include <commonroad_cpp/auxiliaryDefs/interval.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/commonroad_dynamic.pb.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/commonroad_map.pb.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/commonroad_scenario.pb.h>
#include <commonroad_cpp/interfaces/commonroad/protobufFormat/generated/environment_obstacle.pb.h>

#include <google/protobuf/message_lite.h>

class Obstacle;
class PlanningProblem;
class Lanelet;
class IncomingGroup;
class OutgoingGroup;
class CrossingGroup;
class Circle;

namespace ProtobufReader {

using IntegerInterval = ::IntegerInterval;
using FloatInterval = ::FloatInterval;

using IntegerExactOrInterval = std::variant<int, IntegerInterval>;
using FloatExactOrInterval = std::variant<double, FloatInterval>;

using LaneletContainer = std::unordered_map<size_t, std::shared_ptr<Lanelet>>;
using BoundaryContainer = std::unordered_map<size_t, std::shared_ptr<Bound>>;
using TrafficSignContainer = std::unordered_map<size_t, std::shared_ptr<TrafficSign>>;
using TrafficLightContainer = std::unordered_map<size_t, std::shared_ptr<TrafficLight>>;
using IncomingGroupContainer = std::unordered_map<size_t, std::shared_ptr<IncomingGroup>>;
using OutgoingGroupContainer = std::unordered_map<size_t, std::shared_ptr<OutgoingGroup>>;
using CrossingGroupContainer = std::unordered_map<size_t, std::shared_ptr<CrossingGroup>>;

/**
 * Loads a CommonRoadDynamic message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad_dynamic::CommonRoadDynamic loadDynamicProtobufMessage(const std::string &filePath);

/**
 * Loads a CommonRoadMap message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad_map::CommonRoadMap loadMapProtobufMessage(const std::string &filePath);

/**
 * Loads a CommonRoadScenario message from protobuf file.
 *
 * @param filePath File path
 * @return Message
 */
commonroad_scenario::CommonRoadScenario loadScenarioProtobufMessage(const std::string &filePath);

/**
 * Initializes container of lanelets.
 *
 * @param laneletContainer Lanelet container
 * @param commonRoadMapMsg CommonRoad message
 */
void initLaneletContainer(LaneletContainer &laneletContainer, const commonroad_map::CommonRoadMap &commonRoadMapMsg);

/**
 * Initializes container of traffic signs.
 *
 * @param trafficSignContainer Traffic sign container
 * @param commonRoadMapMsg CommonRoad message
 */
void initTrafficSignContainer(TrafficSignContainer &trafficSignContainer,
                              const commonroad_map::CommonRoadMap &commonRoadMapMsg);

/**
 * Initializes container of traffic lights.
 *
 * @param trafficLightContainer Traffic light container
 * @param commonRoadMapMsg CommonRoad message
 */
void initTrafficLightContainer(TrafficLightContainer &trafficLightContainer,
                               const commonroad_map::CommonRoadMap &commonRoadMapMsg);

/**
 * Initializes container of incomingGroups.
 *
 * @param incomingGroupContainer IncomingGroup container
 * @param intersectionMsg CommonRoad message
 */
void initIncomingGroupContainer(IncomingGroupContainer &incomingGroupContainer,
                                const commonroad_map::Intersection &intersectionMsg);

/**
 * Initializes container of outgoingGroups.
 *
 * @param outgoingGroupContainer OutgoingGroup container
 * @param intersectionMsg CommonRoad message
 */
void initOutgoingGroupContainer(OutgoingGroupContainer &outgoingGroupContainer,
                                const commonroad_map::Intersection &intersectionMsg);

/**
 * Initializes container of crossingGroups.
 *
 * @param crossingGroupContainer CrossingGroup container
 * @param intersectionMsg CommonRoad message
 */
void initCrossingGroupContainer(CrossingGroupContainer &crossingGroupContainer,
                                const commonroad_map::Intersection &intersectionMsg);

/**
 * Returns lanelet from container of lanelets.
 *
 * @param laneletId Lanelet id
 * @param laneletContainer Lanelet container
 * @return Lanelet
 */
std::shared_ptr<Lanelet> getLaneletFromContainer(size_t laneletId, LaneletContainer &laneletContainer);

/**
 * Returns traffic sign from container of traffic signs.
 *
 * @param trafficSignId Traffic sign id
 * @param trafficSignContainer Traffic sign container
 * @return Traffic sign
 */
std::shared_ptr<TrafficSign> getTrafficSignFromContainer(size_t trafficSignId,
                                                         TrafficSignContainer &trafficSignContainer);

/**
 * Returns traffic light from container of traffic lights.
 *
 * @param trafficLightId Traffic light id
 * @param trafficLightContainer Traffic light container
 * @return Traffic light
 */
std::shared_ptr<TrafficLight> getTrafficLightFromContainer(size_t trafficLightId,
                                                           TrafficLightContainer &trafficLightContainer);

/**
 * Creates CR scenario from protobuf message "CommonRoad".
 *
 * @param commonRoadDynamicMsg Protobuf message for dynamic information
 * @param commonRoadMapMsg Protobuf message for static information
 * @param commonRoadMapMsg Protobuf message for scenario information
 * @return List of obstacles, road network, and time step
 */
Scenario createCommonRoadFromMessage(const commonroad_dynamic::CommonRoadDynamic &commonRoadDynamicMsg,
                                     const commonroad_map::CommonRoadMap &commonRoadMapMsg,
                                     const commonroad_scenario::CommonRoadScenario &commonRoadScenarioMsg,
                                     int fileGiven);

/**
 * Creates scenario information from protobuf message "ScenarioMetaInformation".
 *
 * @param scenarioMetaInformationMsg Protobuf message
 * @return Benchmark id and step size
 */
std::tuple<std::string, double>
createScenarioMetaInformationFromMessage(const commonroad_common::ScenarioMetaInformation &scenarioMetaInformationMsg);

/**
 * Creates lanelet from protobuf message "Lanelet".
 *
 * @param laneletMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param trafficSignContainer Traffic sign container
 * @param trafficLightContainer Traffic light container
 * @param boundaries Vector of boundaries.
 * @return Lanelet
 */
std::shared_ptr<Lanelet> createLaneletFromMessage(const commonroad_map::Lanelet &laneletMsg,
                                                  LaneletContainer &laneletContainer,
                                                  TrafficSignContainer &trafficSignContainer,
                                                  TrafficLightContainer &trafficLightContainer,
                                                  const std::vector<std::shared_ptr<Bound>> &boundaries,
                                                  const commonroad_map::CommonRoadMap &commonRoadMapMsg);

/**
 * Creates boundary from protobuf message "Bound".
 *
 * @param boundMsg Protobuf message
 * @return Boundary
 */
std::shared_ptr<Bound> createBoundFromMessage(const commonroad_map::Bound &boundMsg);

/**
 * Creates stop line from protobuf message "StopLine".
 *
 * @param stopLineMsg Protobuf message
 * @param trafficSignContainer Traffic sign container
 * @param trafficLightContainer Traffic light container
 * @return Stop line
 */
std::shared_ptr<StopLine> createStopLineFromMessage(const commonroad_map::StopLine &stopLineMsg);

/**
 * Creates traffic sign from protobuf message "TrafficSign".
 *
 * @param trafficSignMsg Protobuf message
 * @param trafficSignContainer Traffic sign container
 * @param country Country ID of map
 * @return Traffic sign
 */
std::shared_ptr<TrafficSign> createTrafficSignFromMessage(const commonroad_map::TrafficSign &trafficSignMsg,
                                                          TrafficSignContainer &trafficSignContainer,
                                                          const std::string &country);

/**
 * Creates traffic sign element from protobuf message "TrafficSignElement".
 *
 * @param trafficSignElementMsg Protobuf message
 * @param country Country ID of map.
 * @return Traffic sign element
 */
std::shared_ptr<TrafficSignElement>
createTrafficSignElementFromMessage(const commonroad_common::TrafficSignElement &trafficSignElementMsg,
                                    const std::string &country);

/**
 * Creates traffic light from protobuf message "TrafficLight".
 *
 * @param trafficLightMsg Protobuf message
 * @param trafficLightContainer Traffic light container
 * @return Traffic light
 */
std::shared_ptr<TrafficLight>
createTrafficLightFromMessage(const commonroad_map::TrafficLight &trafficLightMsg,
                              const commonroad_dynamic::TrafficLightCycle &trafficLightCycleMsg,
                              TrafficLightContainer &trafficLightContainer);

/**
 * Creates traffic light cycle element from protobuf message "CycleElement".
 *
 * @param cycleElementMsg Protobuf message
 * @return Traffic light cycle element
 */
TrafficLightCycleElement createCycleElementFromMessage(const commonroad_dynamic::CycleElement &cycleElementMsg);

/**
 * Creates intersection from protobuf message "Intersection".
 *
 * @param intersectionMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @return Intersection
 */
std::shared_ptr<Intersection> createIntersectionFromMessage(const commonroad_map::Intersection &intersectionMsg,
                                                            LaneletContainer &laneletContainer);

/**
 * Creates incomingGroup from protobuf message "incomingGroup".
 *
 * @param incomingGroupMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param incomingGroupContainer IncomingGroup container
 * @return Incoming
 */
std::shared_ptr<IncomingGroup> createIncomingGroupFromMessage(const commonroad_map::IncomingGroup &incomingGroupMsg,
                                                              LaneletContainer &laneletContainer,
                                                              IncomingGroupContainer &incomingGroupContainer);

/**
 * Creates outgoingGroup from protobuf message "outgoingGroup".
 *
 * @param outgoingGroupMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param outgoingGroupContainer IncomingGroup container
 * @return Outgoing
 */
std::shared_ptr<OutgoingGroup> createOutgoingGroupFromMessage(const commonroad_map::OutgoingGroup &outgoingGroupMsg,
                                                              LaneletContainer &laneletContainer,
                                                              OutgoingGroupContainer &outgoingGroupContainer);

/**
 * Creates crossing group from protobuf message "crossingGroup".
 *
 * @param crossingGroupMsg Protobuf message
 * @param laneletContainer Lanelet container
 * @param crossingGroupContainer CrossingGroup container
 * @return Crossing
 */
std::shared_ptr<CrossingGroup> createCrossingGroupFromMessage(const commonroad_map::CrossingGroup &crossingGroupMsg,
                                                              LaneletContainer &laneletContainer,
                                                              CrossingGroupContainer &crossingGroupContainer);

/**
 * Creates obstacle from protobuf message "EnvironmentObstacle".
 *
 * @param environmentObstacleMsg Protobuf message
 * @return Environment obstacle
 */
std::shared_ptr<Obstacle>
createEnvironmentObstacleFromMessage(const commonroad_map::EnvironmentObstacle &environmentObstacleMsg);

/**
 * Creates obstacle from protobuf message "StaticObstacle".
 *
 * @param staticObstacleMsg Protobuf message
 * @return Static obstacle
 */
std::shared_ptr<Obstacle> createStaticObstacleFromMessage(const commonroad_dynamic::StaticObstacle &staticObstacleMsg);

/**
 * Creates obstacle from protobuf message "DynamicObstacle".
 *
 * @param dynamicObstacleMsg Protobuf message
 * @return Dynamic obstacle
 */
std::shared_ptr<Obstacle>
createDynamicObstacleFromMessage(const commonroad_dynamic::DynamicObstacle &dynamicObstacleMsg);

/**
 * Creates obstacle from protobuf message "PhantomObstacle".
 *
 * @param phantomObstacleMsg Protobuf message
 * @return Phantom obstacle
 */
std::shared_ptr<Obstacle>
createPhantomObstacleFromMessage(const commonroad_dynamic::PhantomObstacle &phantomObstacleMsg);

/**
 * Creates sequence of states from protobuf message "TrajectoryPrediction".
 *
 * @param trajectoryPredictionMsg Protobuf message
 * @return States
 */
std::vector<std::shared_ptr<State>>
createTrajectoryPredictionFromMessage(const commonroad_dynamic::TrajectoryPrediction &trajectoryPredictionMsg);

/**
 * Creates sequence of states from protobuf message "Trajectory".
 *
 * @param trajectoryMsg Protobuf message
 * @return States
 */
std::vector<std::shared_ptr<State>> createTrajectoryFromMessage(const commonroad_dynamic::Trajectory &trajectoryMsg);

/**
 * Creates scenario ID from protobuf message "ScenarioID".
 *
 * @param scenarioIDMsg Protobuf message
 * @return Benchmark id as string
 */
std::string createScenarioIDFromMessage(const commonroad_common::ScenarioID &scenarioIdMsg);

/**
 * Creates scenario ID from protobuf message "MapID".
 *
 * @param mapIdMsg Protobuf message
 * @return Map id as string
 */
std::string createMapIDFromMessage(const commonroad_common::MapID &mapIdMsg);

/**
 * Creates state from protobuf message "State".
 *
 * @param stateMsg Protobuf message
 * @return State
 */
std::shared_ptr<State> createStateFromMessage(const commonroad_common::State &stateMsg);

/**
 * Creates initial state from protobuf message "State".
 *
 * @param stateMsg Protobuf message
 * @return InitialState
 */
std::shared_ptr<InitialState> createInitialStateFromMessage(const commonroad_common::State &stateMsg);

/**
 * Creates signal state from protobuf message "SignalState".
 * @param stateMsg Protobuf message
 * @return State
 */
std::shared_ptr<SignalState> createSignalStateFromMessage(const commonroad_common::SignalState &stateMsg);

/**
 * Creates vertex from protobuf message "Point".
 *
 * @param pointMsg Protobuf message
 * @return Vertex
 */
vertex createPointFromMessage(const commonroad_common::Point &pointMsg);

/**
 * Creates shape from protobuf message "Shape".
 *
 * @param shapeMsg Protobuf message
 * @return Shape
 */
std::unique_ptr<Shape> createShapeFromMessage(const commonroad_common::Shape &shapeMsg);

/**
 * Creates rectangle from protobuf message "Rectangle".
 *
 * @param rectangleMsg Protobuf message
 * @return Rectangle
 */
std::unique_ptr<Rectangle> createRectangleFromMessage(const commonroad_common::Rectangle &rectangleMsg);

/**
 * Creates circle from protobuf message "Circle".
 *
 * @param circleMsg Protobuf message
 * @return Circle
 */
std::unique_ptr<Circle> createCircleFromMessage(const commonroad_common::Circle &circleMsg);

/**
 * Creates integer interval from protobuf message "IntegerInterval".
 *
 * @param integerIntervalMsg Protobuf message
 * @return Integer interval
 */
IntegerInterval createIntegerIntervalFromMessage(const commonroad_common::IntegerInterval &integerIntervalMsg);

/**
 * Creates float interval from protobuf message "FloatInterval".
 *
 * @param floatIntervalMsg Protobuf message
 * @return Float interval
 */
FloatInterval createFloatIntervalFromMessage(const commonroad_common::FloatInterval &floatIntervalMsg);

/**
 * Creates either integer or integer interval from protobuf message "IntegerExactOrInterval".
 *
 * @param integerExactOrIntervalMsg Protobuf message
 * @return Integer or integer interval
 */
IntegerExactOrInterval
createIntegerExactOrInterval(const commonroad_common::IntegerExactOrInterval &integerExactOrIntervalMsg);

/**
 * Creates either float or float interval from protobuf message "FloatExactOrInterval".
 *
 * @param floatExactOrIntervalMsg Protobuf message
 * @return Float or float interval
 */
FloatExactOrInterval createFloatExactOrInterval(const commonroad_common::FloatExactOrInterval &floatExactOrIntervalMsg);

std::shared_ptr<PlanningProblem> createPlanningProblem(const commonroad_scenario::PlanningProblem &planningProblemMsg);

} // namespace ProtobufReader
