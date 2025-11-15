#include "commonroad_cpp/interfaces/commonroad/protobuf_reader.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "commonroad_cpp/goal_state.h"
#include "commonroad_cpp/planning_problem.h"
#include "commonroad_cpp/roadNetwork/intersection/crossing_group.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include "commonroad_cpp/scenario.h"
#include <boost/algorithm/string/case_conv.hpp> // for to_lower_copy
#include <commonroad_cpp/geometry/circle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <memory>
#include <stdexcept>
#include <utility>

commonroad_dynamic::CommonRoadDynamic ProtobufReader::loadDynamicProtobufMessage(const std::string &filePath) {
    std::ifstream pbFile(filePath, std::ios::binary | std::ios::ate);
    std::streamsize size = pbFile.tellg();
    pbFile.seekg(0, std::ios::beg);

    std::vector<char> buffer((size_t)size);
    pbFile.read(buffer.data(), size);

    std::string commonRoadStr(buffer.begin(), buffer.end());

    commonroad_dynamic::CommonRoadDynamic commonRoadMsg = commonroad_dynamic::CommonRoadDynamic();
    commonRoadMsg.ParseFromString(commonRoadStr);

    return commonRoadMsg;
}

commonroad_map::CommonRoadMap ProtobufReader::loadMapProtobufMessage(const std::string &filePath) {
    std::ifstream pbFile(filePath, std::ios::binary | std::ios::ate);
    std::streamsize size = pbFile.tellg();
    pbFile.seekg(0, std::ios::beg);

    std::vector<char> buffer((size_t)size);
    pbFile.read(buffer.data(), size);

    std::string commonRoadStr(buffer.begin(), buffer.end());

    commonroad_map::CommonRoadMap commonRoadMsg = commonroad_map::CommonRoadMap();
    commonRoadMsg.ParseFromString(commonRoadStr);

    return commonRoadMsg;
}

commonroad_scenario::CommonRoadScenario ProtobufReader::loadScenarioProtobufMessage(const std::string &filePath) {
    std::ifstream pbFile(filePath, std::ios::binary | std::ios::ate);
    std::streamsize size = pbFile.tellg();
    pbFile.seekg(0, std::ios::beg);

    std::vector<char> buffer((size_t)size);
    pbFile.read(buffer.data(), size);

    std::string commonRoadStr(buffer.begin(), buffer.end());

    commonroad_scenario::CommonRoadScenario commonRoadMsg = commonroad_scenario::CommonRoadScenario();
    commonRoadMsg.ParseFromString(commonRoadStr);

    return commonRoadMsg;
}

void ProtobufReader::initLaneletContainer(ProtobufReader::LaneletContainer &laneletContainer,
                                          const commonroad_map::CommonRoadMap &commonRoadMapMsg) {
    for (const auto &laneletMsg : commonRoadMapMsg.lanelets())
        laneletContainer.emplace(laneletMsg.lanelet_id(), std::make_shared<Lanelet>());
}

void ProtobufReader::initTrafficSignContainer(ProtobufReader::TrafficSignContainer &trafficSignContainer,
                                              const commonroad_map::CommonRoadMap &commonRoadMapMsg) {
    for (const auto &trafficSignMsg : commonRoadMapMsg.traffic_signs()) {
        trafficSignContainer.emplace(trafficSignMsg.traffic_sign_id(), std::make_shared<TrafficSign>());
    }
}

void ProtobufReader::initTrafficLightContainer(ProtobufReader::TrafficLightContainer &trafficLightContainer,
                                               const commonroad_map::CommonRoadMap &commonRoadMapMsg) {
    for (const auto &trafficLightMsg : commonRoadMapMsg.traffic_lights())
        trafficLightContainer.emplace(trafficLightMsg.traffic_light_id(), std::make_shared<TrafficLight>());
}

void ProtobufReader::initIncomingGroupContainer(IncomingGroupContainer &incomingGroupContainer,
                                                const commonroad_map::Intersection &intersectionMsg) {
    for (const auto &incomingGroupMsg : intersectionMsg.incomings())
        incomingGroupContainer.emplace(incomingGroupMsg.incoming_group_id(), std::make_shared<IncomingGroup>());
}

void ProtobufReader::initOutgoingGroupContainer(OutgoingGroupContainer &outgoingGroupContainer,
                                                const commonroad_map::Intersection &intersectionMsg) {
    for (const auto &outgoingGroupMsg : intersectionMsg.outgoings())
        outgoingGroupContainer.emplace(outgoingGroupMsg.outgoing_group_id(), std::make_shared<OutgoingGroup>());
}

void ProtobufReader::initCrossingGroupContainer(CrossingGroupContainer &crossingGroupContainer,
                                                const commonroad_map::Intersection &intersectionMsg) {
    for (const auto &crossingGroupMsg : intersectionMsg.crossings())
        crossingGroupContainer.emplace(crossingGroupMsg.crossing_group_id(), std::make_shared<CrossingGroup>());
}

std::shared_ptr<Lanelet> ProtobufReader::getLaneletFromContainer(size_t laneletId,
                                                                 ProtobufReader::LaneletContainer &laneletContainer) {
    if (laneletContainer.find(laneletId) != laneletContainer.end())
        return laneletContainer[laneletId];

    return nullptr;
}

std::shared_ptr<TrafficSign>
ProtobufReader::getTrafficSignFromContainer(size_t trafficSignId,
                                            ProtobufReader::TrafficSignContainer &trafficSignContainer) {
    if (trafficSignContainer.find(trafficSignId) != trafficSignContainer.end())
        return trafficSignContainer[trafficSignId];

    return nullptr;
}

std::shared_ptr<TrafficLight>
ProtobufReader::getTrafficLightFromContainer(size_t trafficLightId,
                                             ProtobufReader::TrafficLightContainer &trafficLightContainer) {
    if (trafficLightContainer.find(trafficLightId) != trafficLightContainer.end())
        return trafficLightContainer[trafficLightId];

    return nullptr;
}

Scenario
ProtobufReader::createCommonRoadFromMessage(const commonroad_dynamic::CommonRoadDynamic &commonRoadDynamicMsg,
                                            const commonroad_map::CommonRoadMap &commonRoadMapMsg,
                                            const commonroad_scenario::CommonRoadScenario &commonRoadScenarioMsg,
                                            int fileGiven) {
    std::string benchmarkId;
    double timeStepSize;
    std::tuple<std::string, double> relevantScenarioInformation;
    std::string countryIdName;
    switch (fileGiven) {
    case 7: // scenario, map, dynamic given
        relevantScenarioInformation =
            ProtobufReader::createScenarioMetaInformationFromMessage(commonRoadScenarioMsg.scenario_meta_information());
        benchmarkId = std::get<0>(relevantScenarioInformation);
        timeStepSize = std::get<1>(relevantScenarioInformation);
        countryIdName = benchmarkId.substr(0, benchmarkId.find('_'));
        break;
    case 3: // dynamic and map given
        relevantScenarioInformation =
            ProtobufReader::createScenarioMetaInformationFromMessage(commonRoadDynamicMsg.dynamic_meta_information());
        benchmarkId = std::get<0>(relevantScenarioInformation);
        timeStepSize = std::get<1>(relevantScenarioInformation);
        countryIdName = benchmarkId.substr(0, benchmarkId.find('_'));
        break;
    case 1: // only map given
        timeStepSize = 0;
        benchmarkId = commonRoadMapMsg.map_meta_information().map_id().country_id() + "_" +
                      commonRoadMapMsg.map_meta_information().map_id().map_name() + "-" +
                      std::to_string(commonRoadMapMsg.map_meta_information().map_id().map_id());
        countryIdName = benchmarkId;
        break;
    default:
        throw std::runtime_error("ProtobufReader::createCommonRoadFromMessage: Unsupported file combination given.");
    }

    SupportedTrafficSignCountry countryId = RoadNetwork::matchStringToCountry(countryIdName);

    LaneletContainer laneletContainer;
    initLaneletContainer(laneletContainer, commonRoadMapMsg);

    TrafficSignContainer trafficSignContainer;
    initTrafficSignContainer(trafficSignContainer, commonRoadMapMsg);

    TrafficLightContainer trafficLightContainer;
    initTrafficLightContainer(trafficLightContainer, commonRoadMapMsg);

    std::vector<std::shared_ptr<Bound>> boundaries;
    for (const auto &boundMsg : commonRoadMapMsg.boundaries())
        boundaries.push_back(ProtobufReader::createBoundFromMessage(boundMsg));

    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &laneletMsg : commonRoadMapMsg.lanelets())
        lanelets.push_back(ProtobufReader::createLaneletFromMessage(
            laneletMsg, laneletContainer, trafficSignContainer, trafficLightContainer, boundaries, commonRoadMapMsg));

    std::vector<std::shared_ptr<TrafficSign>> trafficSigns;
    for (const auto &trafficSignMsg : commonRoadMapMsg.traffic_signs())
        trafficSigns.push_back(
            ProtobufReader::createTrafficSignFromMessage(trafficSignMsg, trafficSignContainer, countryIdName));

    std::vector<std::shared_ptr<TrafficLight>> trafficLights;
    for (const auto &trafficLightMsg : commonRoadMapMsg.traffic_lights())
        for (const auto &trafficLightCycleMsg : commonRoadDynamicMsg.traffic_light_cycle()) {
            if (trafficLightMsg.traffic_light_id() == trafficLightCycleMsg.traffic_light_id())
                trafficLights.push_back(ProtobufReader::createTrafficLightFromMessage(
                    trafficLightMsg, trafficLightCycleMsg, trafficLightContainer));
        }

    std::vector<std::shared_ptr<Intersection>> intersections;
    for (const auto &intersectionMsg : commonRoadMapMsg.intersections())
        intersections.push_back(ProtobufReader::createIntersectionFromMessage(intersectionMsg, laneletContainer));

    std::vector<std::shared_ptr<Obstacle>> obstacles;
    for (const auto &dynamicObstacleMsg : commonRoadDynamicMsg.dynamic_obstacles())
        obstacles.push_back(ProtobufReader::createDynamicObstacleFromMessage(dynamicObstacleMsg));

    for (const auto &staticObstacleMsg : commonRoadDynamicMsg.static_obstacles())
        obstacles.push_back(ProtobufReader::createStaticObstacleFromMessage(staticObstacleMsg));

    for (const auto &environmentObstacleMsg : commonRoadMapMsg.environment_obstacles())
        obstacles.push_back(ProtobufReader::createEnvironmentObstacleFromMessage(environmentObstacleMsg));

    for (const auto &phantomObstacleMsg : commonRoadDynamicMsg.phantom_obstacles())
        obstacles.push_back(ProtobufReader::createPhantomObstacleFromMessage(phantomObstacleMsg));

    std::shared_ptr<RoadNetwork> roadNetwork =
        std::make_shared<RoadNetwork>(lanelets, countryId, trafficSigns, trafficLights, intersections);

    for (const auto &intersection : roadNetwork->getIntersections()) {
        intersection->computeMemberLanelets(roadNetwork);
    }

    std::vector<std::shared_ptr<PlanningProblem>> planningProblems;
    for (const auto &planningProblemMsg : commonRoadScenarioMsg.planning_problems()) {
        auto planningProblem = ProtobufReader::createPlanningProblem(planningProblemMsg);
        planningProblems.push_back(planningProblem);
    }

    return Scenario{obstacles, roadNetwork, timeStepSize, planningProblems};
}

std::tuple<std::string, double> ProtobufReader::createScenarioMetaInformationFromMessage(
    const commonroad_common::ScenarioMetaInformation &scenarioMetaInformationMsg) {
    std::string benchmarkId = createScenarioIDFromMessage(scenarioMetaInformationMsg.benchmark_id());
    double timeStepSize = scenarioMetaInformationMsg.time_step_size();

    return std::make_tuple(benchmarkId, timeStepSize);
}

std::string ProtobufReader::createScenarioIDFromMessage(const commonroad_common::ScenarioID &scenarioIDMsg) {
    std::string mapID{createMapIDFromMessage(scenarioIDMsg.map_id())};
    if (scenarioIDMsg.cooperative())
        return "C-" + mapID + "_" + std::to_string(scenarioIDMsg.configuration_id()) + "_" +
               scenarioIDMsg.obstacle_behavior() + "-" + std::to_string(scenarioIDMsg.prediction_id());
    else
        return mapID;
}

std::string ProtobufReader::createMapIDFromMessage(const commonroad_common::MapID &mapIDMsg) {
    return mapIDMsg.country_id() + "_" + mapIDMsg.map_name() + "-" + std::to_string(mapIDMsg.map_id());
}

std::shared_ptr<Lanelet> ProtobufReader::createLaneletFromMessage(
    const commonroad_map::Lanelet &laneletMsg, LaneletContainer &laneletContainer,
    TrafficSignContainer &trafficSignContainer, TrafficLightContainer &trafficLightContainer,
    const std::vector<std::shared_ptr<Bound>> &boundaries, const commonroad_map::CommonRoadMap &commonRoadMapMsg) {
    std::shared_ptr<Lanelet> lanelet = laneletContainer[laneletMsg.lanelet_id()];

    lanelet->setId(laneletMsg.lanelet_id());

    int bordersFound = 0;
    for (const auto &bound : boundaries) {
        if (bound->getBoundaryId() == (size_t)laneletMsg.left_bound()) {
            lanelet->setLeftBorderVertices(bound->getVertices());
            lanelet->setLineMarkingLeft(bound->getLineMarking());
            bordersFound++;
        }
        if (bound->getBoundaryId() == (size_t)laneletMsg.right_bound()) {
            lanelet->setRightBorderVertices(bound->getVertices());
            lanelet->setLineMarkingRight(bound->getLineMarking());
            bordersFound++;
        }
        if (bordersFound == 2) {
            for (size_t vertex_i = 0; vertex_i < lanelet->getLeftBorderVertices().size(); vertex_i++)
                lanelet->addCenterVertex(
                    (lanelet->getLeftBorderVertices()[vertex_i] + lanelet->getRightBorderVertices()[vertex_i]) / 2.);
            break;
        }
    }

    for (size_t laneletId : laneletMsg.successors()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->addSuccessor(getLaneletFromContainer(laneletId, laneletContainer));
    }

    for (size_t laneletId : laneletMsg.predecessors()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->addPredecessor(containerLanelet);
    }

    std::set<LaneletType> laneletTypes;
    for (const auto &laneletType : laneletMsg.lanelet_types()) {
        std::string laneletTypeName =
            commonroad_map::LaneletTypeEnum_LaneletType_Name((commonroad_map::LaneletTypeEnum_LaneletType)laneletType);
        laneletTypes.emplace(lanelet_operations::matchStringToLaneletType(laneletTypeName));
    }
    lanelet->setLaneletTypes(laneletTypes);

    std::set<ObstacleType> userOneWays;
    for (const auto &userOneWay : laneletMsg.user_one_way()) {
        std::string userOneWayName = commonroad_dynamic::ObstacleTypeEnum_ObstacleType_Name(
            (commonroad_dynamic::ObstacleTypeEnum_ObstacleType)userOneWay);
        userOneWays.emplace(obstacle_operations::matchStringToObstacleType(userOneWayName));
    }
    lanelet->setUsersOneWay(userOneWays);

    std::set<ObstacleType> usersBidirectionals;
    for (const auto &usersBidirectional : laneletMsg.user_bidirectional()) {
        std::string userBidirectionalName = commonroad_dynamic::ObstacleTypeEnum_ObstacleType_Name(
            (commonroad_dynamic::ObstacleTypeEnum_ObstacleType)usersBidirectional);
        usersBidirectionals.emplace(obstacle_operations::matchStringToObstacleType(userBidirectionalName));
    }
    lanelet->setUsersBidirectional(usersBidirectionals);

    if (laneletMsg.has_adjacent_left()) {
        auto containerLanelet = getLaneletFromContainer(laneletMsg.adjacent_left(), laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->setLeftAdjacent(containerLanelet, laneletMsg.adjacent_left_opposite_dir());
    }

    if (laneletMsg.has_adjacent_right()) {
        auto containerLanelet = getLaneletFromContainer(laneletMsg.adjacent_right(), laneletContainer);
        if (containerLanelet != nullptr)
            lanelet->setRightAdjacent(containerLanelet, laneletMsg.adjacent_right_opposite_dir());
    }

    if (laneletMsg.has_stop_line()) {
        for (const auto &lane : commonRoadMapMsg.stop_lines()) {
            if (lane.stop_line_id() == laneletMsg.stop_line()) {
                lanelet->setStopLine(ProtobufReader::createStopLineFromMessage(lane));
                break;
            }
        }
    }

    for (size_t trafficSignId : laneletMsg.traffic_sign_refs()) {
        auto containerTrafficSign = getTrafficSignFromContainer(trafficSignId, trafficSignContainer);
        if (containerTrafficSign != nullptr)
            lanelet->addTrafficSign(containerTrafficSign);
    }

    for (size_t trafficLightId : laneletMsg.traffic_light_refs()) {
        auto containerTrafficLight = getTrafficLightFromContainer(trafficLightId, trafficLightContainer);
        if (containerTrafficLight != nullptr)
            lanelet->addTrafficLight(containerTrafficLight);
    }

    lanelet->constructOuterPolygon();
    return lanelet;
}

std::shared_ptr<Bound> ProtobufReader::createBoundFromMessage(const commonroad_map::Bound &boundMsg) {
    std::vector<vertex> points;
    for (const auto &pointMsg : boundMsg.points())
        points.push_back(ProtobufReader::createPointFromMessage(pointMsg));

    std::string lineMarkingName{
        boost::algorithm::to_lower_copy(commonroad_map::LineMarkingEnum_LineMarking_Name(boundMsg.line_marking()))};
    LineMarking lineMarking{lanelet_operations::matchStringToLineMarking(lineMarkingName)};

    return std::make_shared<Bound>(boundMsg.boundary_id(), points, lineMarking);
}

std::shared_ptr<StopLine> ProtobufReader::createStopLineFromMessage(const commonroad_map::StopLine &stopLineMsg) {
    std::shared_ptr<StopLine> stopLine = std::make_shared<StopLine>();

    stopLine->setPoints({{stopLineMsg.start_point().x(), stopLineMsg.start_point().y()},
                         {stopLineMsg.end_point().x(), stopLineMsg.end_point().y()}});

    std::string lineMarkingName{
        boost::algorithm::to_lower_copy(commonroad_map::LineMarkingEnum_LineMarking_Name(stopLineMsg.line_marking()))};
    LineMarking lineMarking{lanelet_operations::matchStringToLineMarking(lineMarkingName)};
    stopLine->setLineMarking(lineMarking);

    return stopLine;
}

std::shared_ptr<TrafficSign>
ProtobufReader::createTrafficSignFromMessage(const commonroad_map::TrafficSign &trafficSignMsg,
                                             TrafficSignContainer &trafficSignContainer, const std::string &country) {
    std::shared_ptr<TrafficSign> trafficSign = trafficSignContainer[trafficSignMsg.traffic_sign_id()];

    trafficSign->setId((size_t)trafficSignMsg.traffic_sign_id());

    std::vector<std::shared_ptr<TrafficSignElement>> trafficSignElements;
    for (const auto &trafficSignElementMsg : trafficSignMsg.traffic_sign_elements())
        trafficSignElements.push_back(
            ProtobufReader::createTrafficSignElementFromMessage(trafficSignElementMsg, country));
    trafficSign->setTrafficSignElements(trafficSignElements);

    if (trafficSignMsg.has_position())
        trafficSign->setPosition(ProtobufReader::createPointFromMessage(trafficSignMsg.position()));

    if (trafficSignMsg.has_virtual_())
        trafficSign->setVirtualElement(trafficSign->isVirtualElement());

    return trafficSign;
}

std::shared_ptr<TrafficSignElement>
ProtobufReader::createTrafficSignElementFromMessage(const commonroad_common::TrafficSignElement &trafficSignElementMsg,
                                                    const std::string &country) {
    std::string elID;
    auto elTyp{TrafficSignNames.at(
        commonroad_common::TrafficSignIDEnum_TrafficSignID_Name(trafficSignElementMsg.element_id()))};
    std::shared_ptr<TrafficSignElement> trafficSignElement = std::make_shared<TrafficSignElement>(elTyp);

    std::vector<std::string> additionalValues(trafficSignElementMsg.additional_values().begin(),
                                              trafficSignElementMsg.additional_values().end());
    trafficSignElement->setAdditionalValues(additionalValues);

    return trafficSignElement;
}

std::shared_ptr<TrafficLight>
ProtobufReader::createTrafficLightFromMessage(const commonroad_map::TrafficLight &trafficLightMsg,
                                              const commonroad_dynamic::TrafficLightCycle &trafficLightCycleMsg,
                                              TrafficLightContainer &trafficLightContainer) {
    std::shared_ptr<TrafficLight> trafficLight = trafficLightContainer[trafficLightMsg.traffic_light_id()];

    trafficLight->setId(trafficLightMsg.traffic_light_id());

    std::vector<TrafficLightCycleElement> cycleElements;
    for (const auto &cycleElementMsg : trafficLightCycleMsg.cycle_elements())
        cycleElements.push_back(ProtobufReader::createCycleElementFromMessage(cycleElementMsg));
    trafficLight->setCycle(cycleElements);

    if (trafficLightMsg.has_position())
        trafficLight->setPosition(ProtobufReader::createPointFromMessage(trafficLightMsg.position()));

    if (trafficLightCycleMsg.has_time_offset())
        trafficLight->setOffset(trafficLightCycleMsg.time_offset());

    if (trafficLightMsg.has_direction()) {
        std::string directionName =
            commonroad_map::TrafficLightDirectionEnum_TrafficLightDirection_Name(trafficLightMsg.direction());
        trafficLight->setDirection(regulatory_elements_utils::matchDirections(directionName));
    }

    if (trafficLightCycleMsg.has_active())
        trafficLight->setActive(trafficLightCycleMsg.active());

    return trafficLight;
}

TrafficLightCycleElement
ProtobufReader::createCycleElementFromMessage(const commonroad_dynamic::CycleElement &cycleElementMsg) {
    TrafficLightCycleElement cycleElement;

    std::string trafficLightStateName =
        commonroad_common::TrafficLightStateEnum_TrafficLightState_Name(cycleElementMsg.color());
    cycleElement.color = TrafficLight::matchTrafficLightState(trafficLightStateName);

    cycleElement.duration = cycleElementMsg.duration();

    return cycleElement;
}

std::shared_ptr<Intersection>
ProtobufReader::createIntersectionFromMessage(const commonroad_map::Intersection &intersectionMsg,
                                              LaneletContainer &laneletContainer) {
    std::shared_ptr<Intersection> intersection = std::make_shared<Intersection>();

    intersection->setId(intersectionMsg.intersection_id());

    IncomingGroupContainer incomingGroupContainer;
    OutgoingGroupContainer outgoingGroupContainer;
    CrossingGroupContainer crossingGroupContainer;
    initIncomingGroupContainer(incomingGroupContainer, intersectionMsg);
    initOutgoingGroupContainer(outgoingGroupContainer, intersectionMsg);
    initCrossingGroupContainer(crossingGroupContainer, intersectionMsg);

    for (const auto &incomingGroupMsg : intersectionMsg.incomings())
        intersection->addIncomingGroup(
            ProtobufReader::createIncomingGroupFromMessage(incomingGroupMsg, laneletContainer, incomingGroupContainer));

    for (const auto &outgoingGroupMsg : intersectionMsg.outgoings())
        intersection->addOutgoingGroup(
            ProtobufReader::createOutgoingGroupFromMessage(outgoingGroupMsg, laneletContainer, outgoingGroupContainer));

    for (const auto &crossingGroupMsg : intersectionMsg.crossings())
        intersection->addCrossingGroup(
            ProtobufReader::createCrossingGroupFromMessage(crossingGroupMsg, laneletContainer, crossingGroupContainer));

    return intersection;
}

std::shared_ptr<IncomingGroup>
ProtobufReader::createIncomingGroupFromMessage(const commonroad_map::IncomingGroup &incomingGroupMsg,
                                               LaneletContainer &laneletContainer,
                                               IncomingGroupContainer &incomingGroupContainer) {
    std::shared_ptr<IncomingGroup> incoming = incomingGroupContainer[incomingGroupMsg.incoming_group_id()];

    incoming->setId(incomingGroupMsg.incoming_group_id());

    if (incomingGroupMsg.has_outgoing_group_id())
        incoming->setOutgoingGroupID(incomingGroupMsg.outgoing_group_id());

    for (size_t laneletId : incomingGroupMsg.incoming_lanelets()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addIncomingLanelet(containerLanelet);
    }

    if (incomingGroupMsg.has_outgoing_group_id())
        incoming->setOutgoingGroupID(incomingGroupMsg.outgoing_group_id());

    for (size_t laneletId : incomingGroupMsg.outgoing_straight()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addStraightOutgoing(containerLanelet);
    }

    for (size_t laneletId : incomingGroupMsg.outgoing_left()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addLeftOutgoing(containerLanelet);
    }

    for (size_t laneletId : incomingGroupMsg.outgoing_right()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            incoming->addRightOutgoing(containerLanelet);
    }

    return incoming;
}

std::shared_ptr<OutgoingGroup>
ProtobufReader::createOutgoingGroupFromMessage(const commonroad_map::OutgoingGroup &outgoingGroupMsg,
                                               LaneletContainer &laneletContainer,
                                               OutgoingGroupContainer &outgoingGroupContainer) {
    std::shared_ptr<OutgoingGroup> outgoing = outgoingGroupContainer[outgoingGroupMsg.outgoing_group_id()];
    outgoing->setId(outgoingGroupMsg.outgoing_group_id());
    if (outgoingGroupMsg.has_incoming_group_id())
        outgoing->setIncomingGroupID(outgoingGroupMsg.incoming_group_id());
    for (size_t laneletId : outgoingGroupMsg.outgoing_lanelets()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            outgoing->addOutgoingLanelet(containerLanelet);
    }
    return outgoing;
}

std::shared_ptr<CrossingGroup>
ProtobufReader::createCrossingGroupFromMessage(const commonroad_map::CrossingGroup &crossingGroupMsg,
                                               LaneletContainer &laneletContainer,
                                               CrossingGroupContainer &crossingGroupContainer) {
    std::shared_ptr<CrossingGroup> crossing = crossingGroupContainer[crossingGroupMsg.crossing_group_id()];
    crossing->setCrossingGroupId(crossingGroupMsg.crossing_group_id());
    if (crossingGroupMsg.has_incoming_group_id())
        crossing->setIncomingGroupID(crossingGroupMsg.incoming_group_id());
    if (crossingGroupMsg.has_outgoing_group_id())
        crossing->setOutgoingGroupID(crossingGroupMsg.outgoing_group_id());
    for (size_t laneletId : crossingGroupMsg.crossing_lanelets()) {
        auto containerLanelet = getLaneletFromContainer(laneletId, laneletContainer);
        if (containerLanelet != nullptr)
            crossing->addCrossingLanelet(containerLanelet);
    }
    return crossing;
}

std::shared_ptr<Obstacle>
ProtobufReader::createStaticObstacleFromMessage(const commonroad_dynamic::StaticObstacle &staticObstacleMsg) {
    std::shared_ptr<Obstacle> staticObstacle = std::make_shared<Obstacle>();

    staticObstacle->setId(staticObstacleMsg.static_obstacle_id());

    staticObstacle->setObstacleRole(ObstacleRole::STATIC);

    staticObstacle->setIsStatic(true);

    staticObstacle->setCurrentState(ProtobufReader::createStateFromMessage(staticObstacleMsg.initial_state()));

    std::string obstacleTypeName =
        commonroad_dynamic::ObstacleTypeEnum_ObstacleType_Name(staticObstacleMsg.obstacle_type());
    staticObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(obstacleTypeName));

    staticObstacle->setGeoShape(ProtobufReader::createShapeFromMessage(staticObstacleMsg.shape()));

    return staticObstacle;
}

std::shared_ptr<Obstacle>
ProtobufReader::createDynamicObstacleFromMessage(const commonroad_dynamic::DynamicObstacle &dynamicObstacleMsg) {
    std::shared_ptr<Obstacle> dynamicObstacle = std::make_shared<Obstacle>();

    dynamicObstacle->setActuatorParameters(ActuatorParameters::vehicleDefaults());
    dynamicObstacle->setSensorParameters(SensorParameters::dynamicDefaults());

    dynamicObstacle->setId(dynamicObstacleMsg.dynamic_obstacle_id());

    dynamicObstacle->setObstacleRole(ObstacleRole::DYNAMIC);

    dynamicObstacle->setIsStatic(false);

    dynamicObstacle->setObstacleType(ObstacleType(dynamicObstacleMsg.obstacle_type()));

    dynamicObstacle->setGeoShape(ProtobufReader::createShapeFromMessage(dynamicObstacleMsg.shape()));

    dynamicObstacle->setCurrentState(ProtobufReader::createStateFromMessage(dynamicObstacleMsg.initial_state()));

    if (dynamicObstacleMsg.has_initial_signal_state())
        dynamicObstacle->setCurrentSignalState(
            ProtobufReader::createSignalStateFromMessage(dynamicObstacleMsg.initial_signal_state()));
    if (dynamicObstacleMsg.has_trajectory_prediction())
        for (const auto &state :
             ProtobufReader::createTrajectoryPredictionFromMessage(dynamicObstacleMsg.trajectory_prediction()))
            dynamicObstacle->appendStateToTrajectoryPrediction(state);
    if (!dynamicObstacleMsg.signal_series().empty())
        for (const auto &state : dynamicObstacleMsg.signal_series())
            dynamicObstacle->appendSignalStateToSeries(ProtobufReader::createSignalStateFromMessage(state));

    return dynamicObstacle;
}

std::shared_ptr<Obstacle> ProtobufReader::createEnvironmentObstacleFromMessage(
    const commonroad_map::EnvironmentObstacle &environmentObstacleMsg) {
    std::shared_ptr<Obstacle> environmentObstacle = std::make_shared<Obstacle>();

    environmentObstacle->setId(environmentObstacleMsg.environment_obstacle_id());

    environmentObstacle->setObstacleRole(ObstacleRole::ENVIRONMENT);

    std::string obstacleTypeName = commonroad_map::EnvironmentObstacleTypeEnum_EnvironmentObstacleType_Name(
        environmentObstacleMsg.obstacle_type());
    environmentObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(obstacleTypeName));

    environmentObstacle->setGeoShape(ProtobufReader::createShapeFromMessage(environmentObstacleMsg.obstacle_shape()));

    return environmentObstacle;
}

std::shared_ptr<Obstacle>
ProtobufReader::createPhantomObstacleFromMessage(const commonroad_dynamic::PhantomObstacle &phantomObstacleMsg) {
    std::shared_ptr<Obstacle> phantomObstacle = std::make_shared<Obstacle>();

    phantomObstacle->setId(phantomObstacleMsg.obstacle_id());

    phantomObstacle->setObstacleRole(ObstacleRole::PHANTOM);

    // TODO: obstacle role and prediction

    return phantomObstacle;
}

std::vector<std::shared_ptr<State>> ProtobufReader::createTrajectoryPredictionFromMessage(
    const commonroad_dynamic::TrajectoryPrediction &trajectoryPredictionMsg) {
    return ProtobufReader::createTrajectoryFromMessage(trajectoryPredictionMsg.trajectory());
}

std::vector<std::shared_ptr<State>>
ProtobufReader::createTrajectoryFromMessage(const commonroad_dynamic::Trajectory &trajectoryMsg) {
    std::vector<std::shared_ptr<State>> trajectory;

    for (const auto &stateMsg : trajectoryMsg.states())
        trajectory.push_back(ProtobufReader::createStateFromMessage(stateMsg));

    return trajectory;
}

std::shared_ptr<State> ProtobufReader::createStateFromMessage(const commonroad_common::State &stateMsg) {
    std::shared_ptr<State> state = std::make_shared<State>();

    IntegerExactOrInterval timeStep = ProtobufReader::createIntegerExactOrInterval(stateMsg.time_step());
    if (timeStep.index() == 0)
        state->setTimeStep((size_t)std::get<int>(timeStep));
    else
        throw std::logic_error("Type of time step is not supported.");

    if (stateMsg.has_point()) {
        vertex vertex = ProtobufReader::createPointFromMessage(stateMsg.point());
        state->setXPosition(vertex.x);
        state->setYPosition(vertex.y);
    }

    if (stateMsg.has_orientation()) {
        FloatExactOrInterval orientation = ProtobufReader::createFloatExactOrInterval(stateMsg.orientation());
        if (orientation.index() == 0)
            state->setGlobalOrientation(std::get<double>(orientation));
    }

    if (stateMsg.has_velocity()) {
        FloatExactOrInterval velocity = ProtobufReader::createFloatExactOrInterval(stateMsg.velocity());
        if (velocity.index() == 0)
            state->setVelocity(std::get<double>(velocity));
    }

    if (stateMsg.has_acceleration()) {
        FloatExactOrInterval acceleration = ProtobufReader::createFloatExactOrInterval(stateMsg.acceleration());
        if (acceleration.index() == 0)
            state->setAcceleration(std::get<double>(acceleration));
    }

    return state;
}

std::shared_ptr<InitialState> ProtobufReader::createInitialStateFromMessage(const commonroad_common::State &stateMsg) {
    if (!stateMsg.has_time_step() || !stateMsg.time_step().has_exact()) {
        throw std::runtime_error{"initial state requires exact time step"};
    }
    auto timeStep = stateMsg.time_step().exact();

    if (!stateMsg.has_point()) {
        throw std::runtime_error{"initial state requires position"};
    }
    auto xPosition = stateMsg.point().x();
    auto yPosition = stateMsg.point().y();

    if (!stateMsg.has_orientation() || !stateMsg.orientation().has_exact()) {
        throw std::runtime_error{"initial state requires exact orientation"};
    }
    auto orientation = stateMsg.orientation().exact();

    if (!stateMsg.has_velocity() || !stateMsg.velocity().has_exact()) {
        throw std::runtime_error{"initial state requires exact velocity"};
    }
    auto velocity = stateMsg.velocity().exact();

    // FIXME: Not required in XML
    if (!stateMsg.has_acceleration() || !stateMsg.acceleration().has_exact()) {
        throw std::runtime_error{"initial state requires exact acceleration"};
    }
    auto acceleration = stateMsg.acceleration().exact();

    if (!stateMsg.has_yaw_rate() || !stateMsg.yaw_rate().has_exact()) {
        throw std::runtime_error{"initial state requires exact yaw rate"};
    }
    auto yawRate = stateMsg.yaw_rate().exact();

    if (!stateMsg.has_slip_angle() || !stateMsg.slip_angle().has_exact()) {
        throw std::runtime_error{"initial state requires exact slip angle"};
    }
    auto slipAngle = stateMsg.slip_angle().exact();

    std::shared_ptr<InitialState> state = std::make_shared<InitialState>(timeStep, xPosition, yPosition, orientation,
                                                                         velocity, acceleration, yawRate, slipAngle);

    return state;
}

std::shared_ptr<SignalState>
ProtobufReader::createSignalStateFromMessage(const commonroad_common::SignalState &stateMsg) {
    std::shared_ptr<SignalState> state = std::make_shared<SignalState>();

    if (stateMsg.has_time_step()) {
        IntegerExactOrInterval timeStep = ProtobufReader::createIntegerExactOrInterval(stateMsg.time_step());
        if (timeStep.index() == 0)
            state->setTimeStep((size_t)std::get<int>(timeStep));
    }
    if (stateMsg.has_horn())
        state->setHorn(true);
    if (stateMsg.indicator_left())
        state->setIndicatorLeft(true);
    if (stateMsg.indicator_right())
        state->setIndicatorRight(true);
    if (stateMsg.has_braking_lights())
        state->setBrakingLights(true);
    if (stateMsg.has_hazard_warning_lights())
        state->setHazardWarningLights(true);
    if (stateMsg.has_flashing_blue_lights())
        state->setFlashingBlueLights(true);

    return state;
}

vertex ProtobufReader::createPointFromMessage(const commonroad_common::Point &pointMsg) {
    vertex vertex;

    vertex.x = pointMsg.x();

    vertex.y = pointMsg.y();

    return vertex;
}

std::unique_ptr<Shape> ProtobufReader::createShapeFromMessage(const commonroad_common::Shape &shapeMsg) {
    std::unique_ptr<Shape> shape;

    if (shapeMsg.has_rectangle())
        shape = ProtobufReader::createRectangleFromMessage(shapeMsg.rectangle());
    else if (shapeMsg.has_circle())
        shape = ProtobufReader::createCircleFromMessage(shapeMsg.circle());

    return shape;
}

std::unique_ptr<Rectangle>
ProtobufReader::createRectangleFromMessage(const commonroad_common::Rectangle &rectangleMsg) {
    std::unique_ptr<Rectangle> rectangle = std::make_unique<Rectangle>();

    rectangle->setLength(rectangleMsg.length());

    rectangle->setWidth(rectangleMsg.width());

    if (rectangleMsg.has_center()) {
        vertex point = ProtobufReader::createPointFromMessage(rectangleMsg.center());
        rectangle->setCenter(point.x, point.y);
    }

    return rectangle;
}

std::unique_ptr<Circle> ProtobufReader::createCircleFromMessage(const commonroad_common::Circle &circleMsg) {
    std::unique_ptr<Circle> circle = std::make_unique<Circle>();

    circle->setRadius(circleMsg.radius());

    if (circleMsg.has_center()) {
        vertex point = ProtobufReader::createPointFromMessage(circleMsg.center());
        circle->setCenter(point.x, point.y);
    }

    return circle;
}

ProtobufReader::IntegerInterval
ProtobufReader::createIntegerIntervalFromMessage(const commonroad_common::IntegerInterval &integerIntervalMsg) {
    return std::make_pair(integerIntervalMsg.start(), integerIntervalMsg.end());
}

ProtobufReader::FloatInterval
ProtobufReader::createFloatIntervalFromMessage(const commonroad_common::FloatInterval &floatIntervalMsg) {
    return std::make_pair(floatIntervalMsg.start(), floatIntervalMsg.end());
}

ProtobufReader::IntegerExactOrInterval ProtobufReader::createIntegerExactOrInterval(
    const commonroad_common::IntegerExactOrInterval &integerExactOrIntervalMsg) {
    IntegerExactOrInterval integerExactOrInterval;

    if (integerExactOrIntervalMsg.has_exact())
        integerExactOrInterval = integerExactOrIntervalMsg.exact();
    else if (integerExactOrIntervalMsg.has_interval())
        integerExactOrInterval = ProtobufReader::createIntegerIntervalFromMessage(integerExactOrIntervalMsg.interval());

    return integerExactOrInterval;
}

ProtobufReader::FloatExactOrInterval
ProtobufReader::createFloatExactOrInterval(const commonroad_common::FloatExactOrInterval &floatExactOrIntervalMsg) {
    FloatExactOrInterval floatExactOrInterval;

    if (floatExactOrIntervalMsg.has_exact())
        floatExactOrInterval = floatExactOrIntervalMsg.exact();
    else if (floatExactOrIntervalMsg.has_interval())
        floatExactOrInterval = ProtobufReader::createFloatIntervalFromMessage(floatExactOrIntervalMsg.interval());

    return floatExactOrInterval;
}

std::shared_ptr<PlanningProblem>
ProtobufReader::createPlanningProblem(const commonroad_scenario::PlanningProblem &planningProblemMsg) {
    auto id = planningProblemMsg.planning_problem_id();

    // TODO: Read scenario tags
    // auto tags = planningProblemMsg.scenario_tags();

    auto initial_state_msg = planningProblemMsg.initial_state();
    auto initial_state = ProtobufReader::createInitialStateFromMessage(initial_state_msg);

    std::vector<GoalState> goal_states;
    auto goal_states_container = planningProblemMsg.goal_states();
    for (const auto &goal_state_msg : goal_states_container) {
        auto state = goal_state_msg.state();

        // Mirror the XML format: Goal state time is always required (velocity and orientation are optional)
        if (!state.has_time_step() || !state.time_step().has_interval()) {
            throw std::runtime_error{"goal state needs to have a time interval"};
        }
        auto time = ProtobufReader::createIntegerIntervalFromMessage(state.time_step().interval());

        std::optional<FloatInterval> velocity = std::nullopt;
        if (!state.has_velocity() || !state.velocity().has_interval()) {
            velocity = ProtobufReader::createFloatIntervalFromMessage(state.velocity().interval());
        }

        std::optional<FloatInterval> orientation = std::nullopt;
        if (!state.has_orientation() || !state.orientation().has_interval()) {
            orientation = ProtobufReader::createFloatIntervalFromMessage(state.orientation().interval());
        }

        std::vector<GoalPosition> goal_positions;
        for (auto lanelet : goal_state_msg.goal_position_lanelets()) {
            goal_positions.push_back(lanelet);
        }
        // TODO: Supports goal shape (instead of lanelet)
        if (goal_positions.empty()) {
            // FIXME: Should a GoalState without lanelets be allowed?
            // throw std::runtime_error{"goal state needs to have at least one goal position lanelet"};
        }

        goal_states.emplace_back(time, velocity, orientation, goal_positions);
    }

    return std::make_shared<PlanningProblem>(id, initial_state, goal_states);
}
