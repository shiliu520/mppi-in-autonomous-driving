#include <iostream>

#include "commonroad_cpp/roadNetwork/intersection/crossing_group.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "commonroad_cpp/world.h"
#include "translate_python_types.h"
#include <commonroad_cpp/auxiliaryDefs/regulatory_elements.h>
#include <commonroad_cpp/geometry/circle.h>
#include <commonroad_cpp/geometry/polygon.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/geometry/shape_group.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/obstacle/occupancy.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <nanobind/stl/list.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <spdlog/spdlog.h>
namespace nb = nanobind;

std::vector<std::shared_ptr<TrafficSign>>
TranslatePythonTypes::convertTrafficSigns(const nb::handle &py_laneletNetwork) {
    std::vector<std::shared_ptr<TrafficSign>> trafficSignContainer;
    const nb::list &py_trafficSigns = py_laneletNetwork.attr("traffic_signs");
    trafficSignContainer.reserve(py_trafficSigns.size()); // Already know the size --> Faster memory allocation

    for (const auto &py_trafficSign : py_trafficSigns) {
        std::shared_ptr<TrafficSign> tempTrafficSign = std::make_shared<TrafficSign>();
        tempTrafficSign->setId(nb::cast<size_t>(py_trafficSign.attr("traffic_sign_id")));
        const nb::list &py_trafficSignElements = py_trafficSign.attr("traffic_sign_elements");
        for (const nb::handle &py_trafficSignElement : py_trafficSignElements) {
            auto trafficSignElementId =
                nb::cast<std::string>(py_trafficSignElement.attr("traffic_sign_element_id").attr("name"));
            std::shared_ptr<TrafficSignElement> newTrafficSignElement{
                std::make_shared<TrafficSignElement>(TrafficSignNames.at(trafficSignElementId))};
            const nb::list &additionalValues = py_trafficSignElement.attr("additional_values");
            std::vector<std::string> additionalValuesList{nb::cast<std::vector<std::string>>(additionalValues)};
            newTrafficSignElement->setAdditionalValues(additionalValuesList);
            tempTrafficSign->addTrafficSignElement(newTrafficSignElement);
        }
        tempTrafficSign->setVirtualElement(nb::cast<bool>(py_trafficSign.attr("virtual")));
        auto py_trafficSignPosition = nb::cast<std::vector<double>>(nb::getattr(py_trafficSign, "position"));
        tempTrafficSign->setPosition({py_trafficSignPosition.at(0), py_trafficSignPosition.at(1)});
        trafficSignContainer.emplace_back(tempTrafficSign);
    }

    return trafficSignContainer;
}

std::vector<TrafficLightCycleElement>
TranslatePythonTypes::convertTrafficLightCycleElements(const nb::handle &py_trafficLightCycle) {
    std::vector<TrafficLightCycleElement> cycle;
    for (const nb::handle &py_cycleElement : py_trafficLightCycle) {
        cycle.push_back(
            {TrafficLight::matchTrafficLightState(nb::cast<std::string>(py_cycleElement.attr("state").attr("value"))),
             nb::cast<size_t>(py_cycleElement.attr("duration"))});
    }
    return cycle;
}

std::vector<std::shared_ptr<TrafficLight>>
TranslatePythonTypes::convertTrafficLights(const nb::handle &py_laneletNetwork) {
    std::vector<std::shared_ptr<TrafficLight>> trafficLightContainer;
    const nb::list &py_trafficLights = py_laneletNetwork.attr("traffic_lights");
    trafficLightContainer.reserve(py_trafficLights.size()); // Already know the size --> Faster memory allocation

    for (const auto &py_trafficLight : py_trafficLights) {
        std::shared_ptr<TrafficLight> tempTrafficLight = std::make_shared<TrafficLight>();
        tempTrafficLight->setId(nb::cast<size_t>(py_trafficLight.attr("traffic_light_id")));
        tempTrafficLight->setOffset(nb::cast<size_t>(py_trafficLight.attr("traffic_light_cycle").attr("time_offset")));
        const nb::list &py_trafficLightCycle = py_trafficLight.attr("traffic_light_cycle").attr("cycle_elements");
        std::vector<TrafficLightCycleElement> cycle{convertTrafficLightCycleElements(py_trafficLightCycle)};
        tempTrafficLight->setCycle(cycle);
        tempTrafficLight->setActive(nb::cast<bool>(py_trafficLight.attr("active")));
        auto py_trafficLightPosition = nb::cast<std::vector<double>>(nb::getattr(py_trafficLight, "position"));
        tempTrafficLight->setPosition({py_trafficLightPosition.at(0), py_trafficLightPosition.at(1)});
        tempTrafficLight->setDirection(regulatory_elements_utils::matchDirections(
            nb::cast<std::string>(py_trafficLight.attr("direction").attr("value"))));
        trafficLightContainer.emplace_back(tempTrafficLight);
    }
    return trafficLightContainer;
}

std::shared_ptr<StopLine>
TranslatePythonTypes::convertStopLine(const nb::handle &py_stopLine,
                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights) {
    std::shared_ptr<StopLine> sl = std::make_shared<StopLine>();
    sl->setLineMarking(lanelet_operations::matchStringToLineMarking(
        nb::cast<std::string>(py_stopLine.attr("_line_marking").attr("value"))));

    auto py_stopLineStartPosition = nb::cast<std::vector<double>>(nb::getattr(py_stopLine, "_start"));
    auto py_stopLineEndPosition = nb::cast<std::vector<double>>(nb::getattr(py_stopLine, "_end"));
    sl->setPoints({{py_stopLineStartPosition.at(0), py_stopLineStartPosition.at(1)},
                   {py_stopLineEndPosition.at(0), py_stopLineEndPosition.at(1)}});
    return sl;
}

std::vector<std::shared_ptr<Lanelet>>
TranslatePythonTypes::convertLanelets(const nb::handle &py_laneletNetwork,
                                      const std::vector<std::shared_ptr<TrafficSign>> &trafficSigns,
                                      const std::vector<std::shared_ptr<TrafficLight>> &trafficLights) {
    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    const nb::list &py_lanelets = py_laneletNetwork.attr("lanelets");
    tempLaneletContainer.reserve(py_lanelets.size()); // Already know the size --> Faster memory allocation
    // all lanelets must be initialized first because they are referencing each other
    for (size_t i = 0; i < py_lanelets.size(); i++) {
        std::shared_ptr<Lanelet> tempLanelet = std::make_shared<Lanelet>();
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    size_t arrayIndex{0};
    // set id of lanelets
    for (nb::handle py_singleLanelet : py_lanelets) {
        tempLaneletContainer[arrayIndex]->setId(nb::cast<size_t>(py_singleLanelet.attr("lanelet_id")));
        arrayIndex++;
    }
    arrayIndex = 0;
    for (nb::handle py_singleLanelet : py_lanelets) {
        // add left vertices
        auto leftVertices = nb::cast<std::vector<std::vector<double>>>(py_singleLanelet.attr("left_vertices"));
        for (const auto &el : leftVertices) {
            vertex newVertex{el.at(0), el.at(1)};
            tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        // add right vertices
        auto py_rightVertices =
            nb::cast<std::vector<std::vector<double>>>(nb::getattr(py_singleLanelet, "right_vertices"));
        for (const auto &el : py_rightVertices) {
            vertex newVertex{el.at(0), el.at(1)};
            tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
        }
        // add users one way
        const nb::set &py_laneletUserOneWay = nb::cast<nb::set>(py_singleLanelet.attr("user_one_way"));
        std::set<ObstacleType> usersOneWay;
        for (nb::handle py_user : py_laneletUserOneWay)
            usersOneWay.insert(
                obstacle_operations::matchStringToObstacleType(nb::cast<std::string>(py_user.attr("value"))));
        tempLaneletContainer[arrayIndex]->setUsersOneWay(usersOneWay);
        // add users bidirectional
        const nb::set &py_laneletUserBidirectional = nb::cast<nb::set>(py_singleLanelet.attr("user_bidirectional"));
        std::set<ObstacleType> usersBidirectional;
        for (nb::handle py_user : py_laneletUserBidirectional)
            usersBidirectional.insert(
                obstacle_operations::matchStringToObstacleType(nb::cast<std::string>(py_user.attr("value"))));
        tempLaneletContainer[arrayIndex]->setUsersBidirectional(usersBidirectional);
        // add lanelet types
        const nb::set &py_laneletTypes = nb::cast<nb::set>(py_singleLanelet.attr("lanelet_type"));
        std::set<LaneletType> laneletTypes;
        for (nb::handle py_type : py_laneletTypes)
            laneletTypes.insert(
                lanelet_operations::matchStringToLaneletType(nb::cast<std::string>(py_type.attr("value"))));
        tempLaneletContainer[arrayIndex]->setLaneletTypes(laneletTypes);
        // set line markings
        tempLaneletContainer[arrayIndex]->setLineMarkingLeft(lanelet_operations::matchStringToLineMarking(
            nb::cast<std::string>(py_singleLanelet.attr("line_marking_left_vertices").attr("value"))));
        tempLaneletContainer[arrayIndex]->setLineMarkingRight(lanelet_operations::matchStringToLineMarking(
            nb::cast<std::string>(py_singleLanelet.attr("line_marking_right_vertices").attr("value"))));
        // set successors
        nb::object py_successors = py_singleLanelet.attr("successor");
        for (nb::handle py_item : py_successors) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == nb::cast<size_t>(py_item)) {
                    tempLaneletContainer[arrayIndex]->addSuccessor(la);
                    break;
                }
            }
        }
        // set predecessors
        nb::object py_predecessors = py_singleLanelet.attr("predecessor");
        for (nb::handle py_item : py_predecessors) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == nb::cast<size_t>(py_item)) {
                    tempLaneletContainer[arrayIndex]->addPredecessor(la);
                    break;
                }
            }
        }
        // add adjacent left
        nb::object py_adjLeft = py_singleLanelet.attr("adj_left");
        if (!py_adjLeft.is_none()) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == nb::cast<size_t>(py_adjLeft)) {
                    if (nb::cast<bool>(py_singleLanelet.attr("adj_left_same_direction"))) // same direction
                        tempLaneletContainer[arrayIndex]->setLeftAdjacent(la, false);
                    else // opposite direction
                        tempLaneletContainer[arrayIndex]->setLeftAdjacent(la, true);
                    break;
                }
            }
        }
        // add adjacent right
        nb::object py_adjRight = py_singleLanelet.attr("adj_right");
        if (!py_adjRight.is_none()) {
            for (const auto &la : tempLaneletContainer) {
                if (la->getId() == nb::cast<size_t>(py_adjRight)) {
                    if (nb::cast<bool>(py_singleLanelet.attr("adj_right_same_direction"))) // same direction
                        tempLaneletContainer[arrayIndex]->setRightAdjacent(la, false);
                    else // opposite direction
                        tempLaneletContainer[arrayIndex]->setRightAdjacent(la, true);
                    break;
                }
            }
        }
        // add traffic signs
        nb::object py_trafficSigns = py_singleLanelet.attr("_traffic_signs");
        for (const auto &sign : trafficSigns) {
            for (const auto &py_sign : py_trafficSigns)
                if (sign->getId() == nb::cast<size_t>(py_sign)) {
                    tempLaneletContainer[arrayIndex]->addTrafficSign(sign);
                    break;
                }
        }
        // add traffic light
        nb::object py_trafficLights = py_singleLanelet.attr("_traffic_lights");
        for (const auto &light : trafficLights) {
            for (const auto &py_light : py_trafficLights)
                if (light->getId() == nb::cast<size_t>(py_light)) {
                    tempLaneletContainer[arrayIndex]->addTrafficLight(light);
                    break;
                }
        }
        nb::handle py_stopLine = py_singleLanelet.attr("stop_line");
        if (!py_stopLine.is_none())
            tempLaneletContainer[arrayIndex]->setStopLine(convertStopLine(py_stopLine, trafficSigns, trafficLights));
        tempLaneletContainer[arrayIndex]->createCenterVertices();
        tempLaneletContainer[arrayIndex]->constructOuterPolygon();
        arrayIndex++;
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>>
TranslatePythonTypes::convertIntersections(const nb::handle &py_laneletNetwork,
                                           const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempIntersectionContainer{};
    const nb::list &py_intersection_list = nb::cast<nb::list>(py_laneletNetwork.attr("intersections"));
    size_t n = py_intersection_list.size();
    tempIntersectionContainer.reserve(n); // Already know the size --> Faster memory allocation
    // all intersections must be initialized first
    for (size_t i = 0; i < py_intersection_list.size(); i++) {
        std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
        tempIntersectionContainer.emplace_back(tempIntersection);
    }

    size_t intersectionIndex{0};
    for (const auto &py_intersection : py_intersection_list) {
        std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
        tempIntersection->setId(nb::cast<size_t>(py_intersection.attr("intersection_id")));
        std::vector<std::shared_ptr<IncomingGroup>> incomings;
        incomings.reserve(nb::cast<nb::list>(py_intersection.attr("incomings")).size());
        for (const auto &py_incoming : nb::cast<nb::list>(py_intersection.attr("incomings"))) {
            std::shared_ptr<IncomingGroup> tempIncoming = std::make_shared<IncomingGroup>();
            tempIncoming->setId(nb::cast<size_t>(py_incoming.attr("incoming_id")));
            incomings.emplace_back(tempIncoming);
        }
        size_t incomingIndex{0};
        for (const auto &py_incoming : nb::cast<nb::list>(py_intersection.attr("incomings"))) {
            // incoming lanelets
            auto py_incomingLanelets = nb::cast<nb::set>(py_incoming.attr("incoming_lanelets"));
            std::vector<std::shared_ptr<Lanelet>> incomingLanelets;
            for (const auto &incomingLaneletId : py_incomingLanelets) {
                size_t incId{nb::cast<size_t>(incomingLaneletId)};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        incomingLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomingIndex]->setIncomingLanelets(incomingLanelets);
            // successor right lanelets
            auto py_outgoingRight = nb::cast<nb::set>(py_incoming.attr("outgoing_right"));
            std::vector<std::shared_ptr<Lanelet>> outgoingRightLanelets;
            for (const auto &outgoingRightLaneletId : py_outgoingRight) {
                size_t incId{nb::cast<size_t>(outgoingRightLaneletId)};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        outgoingRightLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomingIndex]->setRightOutgoings(outgoingRightLanelets);
            // successor left lanelets
            auto py_outgoingLeft = nb::cast<nb::set>(py_incoming.attr("outgoing_left"));
            std::vector<std::shared_ptr<Lanelet>> outgoingLeftLanelets;
            for (const auto &outgoingLeftLaneletId : py_outgoingLeft) {
                size_t incId{nb::cast<size_t>(outgoingLeftLaneletId)};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        outgoingLeftLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomingIndex]->setLeftOutgoings(outgoingLeftLanelets);
            // successor straight lanelets
            auto py_outgoingsStraight = nb::cast<nb::set>(py_incoming.attr("outgoing_straight"));
            std::vector<std::shared_ptr<Lanelet>> outgoingsStraightLanelets;
            for (const auto &outgoingsStraightLaneletId : py_outgoingsStraight) {
                size_t incId{nb::cast<size_t>(outgoingsStraightLaneletId)};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        outgoingsStraightLanelets.push_back(la);
                        break;
                    }
                }
            }
            incomings[incomingIndex]->setStraightOutgoings(outgoingsStraightLanelets);

            if (!py_incoming.attr("outgoing_group_id").is_none())
                incomings[incomingIndex]->setOutgoingGroupID(nb::cast<size_t>(py_incoming.attr("outgoing_group_id")));
            incomingIndex++;
        }
        tempIntersectionContainer[intersectionIndex]->setIncomingGroups(incomings);
        // outgoingGroups
        std::vector<std::shared_ptr<OutgoingGroup>> outgoings;
        outgoings.reserve(nb::cast<nb::list>(py_intersection.attr("outgoings")).size());
        for (const auto &py_outgoing : nb::cast<nb::list>(py_intersection.attr("outgoings"))) {
            std::shared_ptr<OutgoingGroup> tempOutgoing = std::make_shared<OutgoingGroup>();
            tempOutgoing->setId(nb::cast<size_t>(py_outgoing.attr("outgoing_id")));
            outgoings.emplace_back(tempOutgoing);
        }
        size_t outgoingIndex{0};
        for (const auto &py_outgoing : nb::cast<nb::list>(py_intersection.attr("outgoings"))) {
            // Outgoing lanelets
            auto py_outgoingLanelets = nb::cast<nb::set>(py_outgoing.attr("outgoing_lanelets"));
            std::vector<std::shared_ptr<Lanelet>> outgoingLanelets;
            for (const auto &outgoingLaneletId : py_outgoingLanelets) {
                size_t incId{nb::cast<size_t>(outgoingLaneletId)};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        outgoingLanelets.push_back(la);
                        break;
                    }
                }
            }
            if (!py_outgoing.attr("incoming_group_id").is_none())
                outgoings[outgoingIndex]->setIncomingGroupID(nb::cast<size_t>(py_outgoing.attr("incoming_group_id")));

            outgoings[outgoingIndex]->setOutgoingLanelets(outgoingLanelets);
            outgoingIndex++;
        }
        tempIntersectionContainer[intersectionIndex]->setOutgoingGroups(outgoings);
        // crossingGroups
        std::vector<std::shared_ptr<CrossingGroup>> crossings;
        crossings.reserve(nb::cast<nb::list>(py_intersection.attr("crossings")).size());
        for (const auto &py_crossing : py_intersection.attr("crossings")) {
            std::shared_ptr<CrossingGroup> tempCrossing = std::make_shared<CrossingGroup>();
            tempCrossing->setCrossingGroupId(nb::cast<size_t>(py_crossing.attr("crossing_id")));
            crossings.emplace_back(tempCrossing);
        }
        size_t crossingIndex{0};
        for (const auto &py_crossing : py_intersection.attr("crossings")) {
            // Crossing lanelets
            auto py_crossingLanelets = py_crossing.attr("crossing_lanelets");
            std::vector<std::shared_ptr<Lanelet>> crossingLanelets;
            for (const auto &crossingLaneletId : py_crossingLanelets) {
                size_t incId{nb::cast<size_t>(crossingLaneletId)};
                for (const auto &la : lanelets) {
                    if (la->getId() == incId) {
                        crossingLanelets.push_back(la);
                        break;
                    }
                }
            }
            if (!py_crossing.attr("incoming_group_id").is_none())
                crossings[crossingIndex]->setIncomingGroupID(nb::cast<size_t>(py_crossing.attr("incoming_group_id")));
            if (!py_crossing.attr("outgoing_group_id").is_none())
                crossings[crossingIndex]->setOutgoingGroupID(nb::cast<size_t>(py_crossing.attr("outgoing_group_id")));

            crossings[crossingIndex]->setCrossingGroupLanelets(crossingLanelets);
            crossingIndex++;
        }
        tempIntersectionContainer[intersectionIndex]->setCrossingGroups(crossings);
        intersectionIndex++;
    }
    return tempIntersectionContainer;
}

std::shared_ptr<State> createInitialState(nb::handle py_singleObstacle) {
    // TODO add support for uncertain states
    std::shared_ptr<State> initialState = std::make_shared<State>();

    auto xPos = nb::cast<std::vector<double>>(py_singleObstacle.attr("initial_state").attr("position"));
    auto yPos = nb::cast<std::vector<double>>(py_singleObstacle.attr("initial_state").attr("position"));
    auto timeStep = nb::cast<size_t>(py_singleObstacle.attr("initial_state").attr("time_step"));
    initialState->setXPosition(xPos.at(0));
    initialState->setYPosition(yPos.at(1));
    initialState->setTimeStep(timeStep);
    initialState->setGlobalOrientation(nb::cast<double>(py_singleObstacle.attr("initial_state").attr("orientation")));
    if (nb::hasattr(py_singleObstacle.attr("initial_state"), "velocity"))
        initialState->setVelocity(nb::cast<double>(py_singleObstacle.attr("initial_state").attr("velocity")));
    if (nb::hasattr(py_singleObstacle.attr("initial_state"), "acceleration"))
        initialState->setAcceleration(nb::cast<double>(py_singleObstacle.attr("initial_state").attr("acceleration")));

    return initialState;
}

std::shared_ptr<Shape> extractOccupancyShape(nb::handle py_occupancyShape) {
    std::string commonroadShape{nb::cast<std::string>(inst_name(py_occupancyShape))};
    if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "Rectangle") {
        auto length{nb::cast<double>(py_occupancyShape.attr("length"))};
        auto width{nb::cast<double>(py_occupancyShape.attr("width"))};
        return std::make_shared<Rectangle>(length, width);
    } else if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "Circle") {
        auto radius{nb::cast<double>(py_occupancyShape.attr("radius"))};
        return std::make_shared<Circle>(radius);
    } else if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "Polygon") {
        auto verticesRaw = nb::cast<std::vector<std::vector<double>>>(py_occupancyShape.attr("vertices"));
        std::vector<vertex> vertices;
        for (const auto &el : verticesRaw)
            vertices.push_back({el.at(0), el.at(1)});
        return std::make_shared<Polygon>(vertices);
    } else if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "ShapeGroup") {
        std::vector<std::shared_ptr<Shape>> shapes;
        for (const auto &shape : py_occupancyShape.attr("shapes"))
            shapes.push_back(extractOccupancyShape(shape));
        return std::make_shared<ShapeGroup>(shapes);
    } else
        spdlog::error("extractOccupancyShape: Unknown obstacle shape.");
}

std::unique_ptr<Shape> extractObstacleShape(nb::handle py_obstacleShape) {
    std::string commonroadShape{nb::cast<std::string>(inst_name(py_obstacleShape))};
    if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "Rectangle") {
        auto length{nb::cast<double>(py_obstacleShape.attr("length"))};
        auto width{nb::cast<double>(py_obstacleShape.attr("width"))};
        return std::make_unique<Rectangle>(length, width);
    } else if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "Circle") {
        auto radius{nb::cast<double>(py_obstacleShape.attr("radius"))};
        return std::make_unique<Circle>(radius);
    } else if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "Polygon") {
        std::vector<vertex> vertices;
        for (const auto &vertex : py_obstacleShape.attr("vertices"))
            vertices.push_back({nb::cast<double>(vertex[0]), nb::cast<double>(vertex[1])});
        return std::make_unique<Polygon>(vertices);
    } else if (commonroadShape.substr(commonroadShape.find_last_of('.') + 1) == "ShapeGroup") {
        std::vector<std::shared_ptr<Shape>> shapes;
        for (const auto &shape : py_obstacleShape.attr("shapes"))
            shapes.push_back(extractOccupancyShape(shape));
        return std::make_unique<ShapeGroup>(shapes);
    } else
        spdlog::error("extractObstacleShape: Unknown obstacle shape.");
}

std::shared_ptr<Obstacle> createCommonObstaclePart(nb::handle py_singleObstacle) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
    tempObstacle->setId(nb::cast<size_t>(py_singleObstacle.attr("obstacle_id")));
    tempObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(
        nb::cast<std::string>(py_singleObstacle.attr("obstacle_type").attr("value"))));
    nb::handle py_obstacleShape = py_singleObstacle.attr("obstacle_shape");
    tempObstacle->setGeoShape(extractObstacleShape(py_obstacleShape));
    tempObstacle->setCurrentState(createInitialState(py_singleObstacle));

    return tempObstacle;
}

std::shared_ptr<State> TranslatePythonTypes::extractState(nb::handle py_state) {
    auto state{std::make_shared<State>()};
    state->setXPosition(nb::cast<std::vector<double>>(py_state.attr("position")).at(0));
    state->setYPosition(nb::cast<std::vector<double>>(py_state.attr("position")).at(1));
    state->setGlobalOrientation(nb::cast<double>(py_state.attr("orientation")));
    if (nb::hasattr(py_state, "velocity"))
        state->setVelocity(nb::cast<double>(py_state.attr("velocity")));
    state->setTimeStep(nb::cast<size_t>(py_state.attr("time_step")));
    if (nb::hasattr(py_state, "acceleration"))
        state->setAcceleration(nb::cast<double>(py_state.attr("acceleration")));

    return state;
}

std::shared_ptr<Occupancy> TranslatePythonTypes::extractOccupancy(nb::handle py_occupancy) {
    auto occ{std::make_shared<Occupancy>()};
    occ->setShape(extractOccupancyShape(py_occupancy.attr("shape")));
    occ->setTimeStep(nb::cast<size_t>(py_occupancy.attr("time_step")));
    return occ;
}

std::shared_ptr<SignalState> extractSignalState(nb::handle py_state) {
    auto state{std::make_shared<SignalState>()};
    state->setTimeStep(nb::cast<size_t>(py_state.attr("time_step")));
    if (nb::hasattr(py_state, "horn"))
        state->setHorn(nb::cast<bool>(py_state.attr("horn")));
    if (nb::hasattr(py_state, "indicator_left"))
        state->setHorn(nb::cast<bool>(py_state.attr("indicator_left")));
    if (nb::hasattr(py_state, "indicator_rightorn"))
        state->setHorn(nb::cast<bool>(py_state.attr("indicator_right")));
    if (nb::hasattr(py_state, "braking_lights"))
        state->setHorn(nb::cast<bool>(py_state.attr("braking_lights")));
    if (nb::hasattr(py_state, "hazard_warning_lights"))
        state->setHorn(nb::cast<bool>(py_state.attr("hazard_warning_lights")));
    if (nb::hasattr(py_state, "flashing_blue_lights"))
        state->setHorn(nb::cast<bool>(py_state.attr("flashing_blue_lights")));
    return state;
}

std::shared_ptr<Obstacle> TranslatePythonTypes::createDynamicObstacle(nb::handle py_singleObstacle) {
    std::shared_ptr<Obstacle> tempObstacle = createCommonObstaclePart(py_singleObstacle);
    if (nb::hasattr(py_singleObstacle.attr("prediction"), "trajectory"))
        for (const auto &py_state : py_singleObstacle.attr("prediction").attr("trajectory").attr("state_list"))
            tempObstacle->appendStateToTrajectoryPrediction(extractState(py_state));
    else if (nb::hasattr(py_singleObstacle.attr("prediction"), "occupancy_set"))
        for (const auto &py_occupancy : py_singleObstacle.attr("prediction").attr("occupancy_set"))
            tempObstacle->appendOccupancyToSetBasedPrediction(extractOccupancy(py_occupancy));
    else
        spdlog::error(
            "TranslatePythonTypes::createDynamicObstacle: Unknown prediction type or no prediction. Obstacle ID: " +
            std::to_string(tempObstacle->getId()));
    if (nb::hasattr(py_singleObstacle, "initial_signal_state") and
        !py_singleObstacle.attr("initial_signal_state").is_none())
        tempObstacle->setCurrentSignalState(extractSignalState(py_singleObstacle.attr("initial_signal_state")));
    if (nb::hasattr(py_singleObstacle, "signal_series") and !py_singleObstacle.attr("signal_series").is_none())
        for (const auto &py_state : py_singleObstacle.attr("signal_series"))
            tempObstacle->appendSignalStateToSeries(extractSignalState(py_state));
    if (nb::hasattr(py_singleObstacle, "history") and !py_singleObstacle.attr("history").is_none())
        for (const auto &py_state : py_singleObstacle.attr("history"))
            tempObstacle->appendStateToHistory(extractState(py_state));
    return tempObstacle;
}

std::shared_ptr<Obstacle> TranslatePythonTypes::createStaticObstacle(nb::handle py_singleObstacle) {
    std::shared_ptr<Obstacle> tempObstacle = createCommonObstaclePart(py_singleObstacle);
    tempObstacle->setIsStatic(true);
    return tempObstacle;
}

std::vector<std::shared_ptr<Obstacle>>
TranslatePythonTypes::convertObstacles(const nb::list &py_obstacle_list, const WorldParameters &worldParams, bool ego) {
    std::vector<std::shared_ptr<Obstacle>> tempObstacleContainer{};

    size_t arrayIndex{0};
    for (nb::handle py_singleObstacle : py_obstacle_list) {
        std::string obstacleRole{nb::cast<std::string>(py_singleObstacle.attr("obstacle_role").attr("value"))};
        if (obstacleRole == "dynamic" or obstacleRole == "static") {
            std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
            tempObstacleContainer.emplace_back(tempObstacle);
        }
        if (obstacleRole == "dynamic")
            tempObstacleContainer[arrayIndex] = createDynamicObstacle(py_singleObstacle);
        else if (obstacleRole == "static")
            tempObstacleContainer[arrayIndex] = createStaticObstacle(py_singleObstacle);
        arrayIndex++;
    }

    if (!worldParams.hasDefaultParams()) {
        for (const auto &obs : tempObstacleContainer) {
            if (ego)
                obs->setActuatorParameters(worldParams.getActuatorParamsEgo());
            else
                obs->setActuatorParameters(worldParams.getActuatorParamsObstacles());
            obs->setSensorParameters(worldParams.getSensorParams());
            obs->setRoadNetworkParameters(worldParams.getRoadNetworkParams());
            obs->setTimeParameters(worldParams.getTimeParams());
        }
    }

    return tempObstacleContainer;
}
