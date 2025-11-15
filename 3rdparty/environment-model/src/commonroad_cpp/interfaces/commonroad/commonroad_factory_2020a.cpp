#include <cstring>

#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/planning_problem.h"
#include "commonroad_cpp/roadNetwork/intersection/crossing_group.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_factory_2020a.h"
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

// MSVC compatibility
#ifdef _MSC_VER
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#endif

double CommonRoadFactory2020a::getTimeStepSize() {
    return doc->child("commonRoad").attribute("timeStepSize").as_double();
}

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2020a::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with dynamic and static obstacles
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "dynamicObstacle")) == 0) {
            XMLReader::createDynamicObstacle(obstacleList, roadElements);
        } else if ((strcmp(roadElements.name(), "staticObstacle")) == 0) {
            XMLReader::extractStaticObstacle(obstacleList, roadElements);
        }
        // TODO add environmental and phantom obstacles
    }
    return obstacleList;
}

std::vector<std::shared_ptr<Lanelet>>
CommonRoadFactory2020a::createLanelets(std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                       std::vector<std::shared_ptr<TrafficLight>> trafficLights) {

    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");
    XMLReader::initializeLanelets(tempLaneletContainer, commonRoad);

    // get the other values of the lanelets
    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "lanelet")) == 0) {
            std::set<LaneletType> laneletType;
            std::set<ObstacleType> userOneWay;
            std::set<ObstacleType> userBidirectional;
            for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
                // set left bound
                if ((strcmp(child.name(), "leftBound")) == 0) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "leftBound");
                    continue;
                }
                // set right bound
                if ((strcmp(child.name(), "rightBound")) == 0) {
                    XMLReader::extractLaneletBoundary(tempLaneletContainer, arrayIndex, child, "rightBound");
                    continue;
                }
                // set successor lanelets
                if ((strcmp(child.name(), "successor")) == 0) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "successor");
                    continue;
                }
                // set predecessor lanelets
                if ((strcmp(child.name(), "predecessor")) == 0) {
                    XMLReader::extractLaneletPreSuc(tempLaneletContainer, arrayIndex, child, "predecessor");
                    continue;
                }
                // set left adjacent lanelets
                if ((strcmp(child.name(), "adjacentLeft")) == 0) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentLeft");
                    continue;
                }
                // set right adjacent lanelets
                if ((strcmp(child.name(), "adjacentRight")) == 0) {
                    XMLReader::extractLaneletAdjacency(tempLaneletContainer, arrayIndex, child, "adjacentRight");
                    continue;
                }
                // set lanelet type
                if ((strcmp(child.name(), "laneletType")) == 0)
                    laneletType.insert(lanelet_operations::matchStringToLaneletType(child.first_child().value()));
                // set user one way
                if ((strcmp(child.name(), "userOneWay")) == 0)
                    userOneWay.insert(obstacle_operations::matchStringToObstacleType(child.first_child().value()));
                // set user bidirectional
                if ((strcmp(child.name(), "userBidirectional")) == 0)
                    userBidirectional.insert(
                        obstacle_operations::matchStringToObstacleType((child.first_child().value())));
                // add traffic signs to temporary list
                if ((strcmp(child.name(), "trafficSignRef")) == 0) {
                    for (const auto &sign : trafficSigns) {
                        if (child.attribute("ref").as_ullong() == sign->getId()) {
                            tempLaneletContainer[arrayIndex]->addTrafficSign(sign);
                        }
                    }
                }
                // add traffic lights to temporary list
                if ((strcmp(child.name(), "trafficLightRef")) == 0) {
                    for (const auto &light : trafficLights) {
                        if (child.attribute("ref").as_ullong() == light->getId()) {
                            tempLaneletContainer[arrayIndex]->addTrafficLight(light);
                        }
                    }
                }
                // set stop line
                if ((strcmp(child.name(), "stopLine")) == 0) {
                    std::vector<vertex> points;
                    std::shared_ptr<StopLine> stopL = std::make_shared<StopLine>();
                    for (pugi::xml_node elem = child.first_child(); elem != nullptr; elem = elem.next_sibling()) {
                        if ((strcmp(elem.name(), "point")) == 0) {
                            vertex newVertex{};
                            newVertex.x = elem.child("x").text().as_double();
                            newVertex.y = elem.child("y").text().as_double();
                            points.push_back(newVertex);
                        }
                        if ((strcmp(elem.name(), "lineMarking")) == 0)
                            stopL->setLineMarking(
                                lanelet_operations::matchStringToLineMarking(elem.first_child().value()));
                        /*if ((strcmp(elem.name(), "trafficSignRef")) == 0) {
                            for (const auto &sign : trafficSigns) {
                                if (child.attribute("ref").as_ullong() == sign->getId()) {
                                    stopL->addTrafficSign(sign);
                                }
                            }
                        }
                        if ((strcmp(elem.name(), "trafficLightRef")) == 0) {
                            for (const auto &light : trafficLights) {
                                if (child.attribute("ref").as_ullong() == light->getId()) {
                                    stopL->addTrafficLight(light);
                                }
                            }
                        }*/
                    }
                    if (!points.empty())
                        stopL->setPoints({points.at(0), points.at(1)});
                    else {
                        stopL->setPoints({tempLaneletContainer[arrayIndex]->getLeftBorderVertices().back(),
                                          tempLaneletContainer[arrayIndex]->getRightBorderVertices().back()});
                    }
                    tempLaneletContainer[arrayIndex]->setStopLine(stopL);
                }
            }
            tempLaneletContainer[arrayIndex]->createCenterVertices();
            tempLaneletContainer[arrayIndex]->constructOuterPolygon();
            tempLaneletContainer[arrayIndex]->setLaneletTypes(laneletType);
            tempLaneletContainer[arrayIndex]->setUsersOneWay(userOneWay);
            tempLaneletContainer[arrayIndex]->setUsersBidirectional(userBidirectional);
            arrayIndex++;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficSign>> CommonRoadFactory2020a::createTrafficSigns() {
    std::vector<std::shared_ptr<TrafficSign>> temptrafficSignContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    std::string benchmarkID{doc->child("commonRoad").attribute("benchmarkID").value()};
    auto country = RoadNetwork::matchStringToCountry(benchmarkID.substr(0, 3));

    // get the number of traffic signs in scenario
    size_t numSigns{static_cast<size_t>(
        std::distance(commonRoad.children("trafficSign").begin(), commonRoad.children("trafficSign").end()))};
    temptrafficSignContainer.clear();
    temptrafficSignContainer.reserve(numSigns); // Size already known --> Faster memory allocation

    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        // get traffic signs
        if ((strcmp(roadElements.name(), "trafficSign")) == 0) {
            std::shared_ptr<TrafficSign> tempTrafficSign = std::make_shared<TrafficSign>();
            temptrafficSignContainer.emplace_back(tempTrafficSign);
            temptrafficSignContainer[arrayIndex]->setId(roadElements.first_attribute().as_ullong());
            for (pugi::xml_node trafficSignChildElement = roadElements.first_child();
                 trafficSignChildElement != nullptr; trafficSignChildElement = trafficSignChildElement.next_sibling()) {
                // get traffic sign elements
                if ((strcmp(trafficSignChildElement.name(), "trafficSignElement")) == 0) {
                    std::string trafficSignId = trafficSignChildElement.first_child().first_child().value();
                    auto newTrafficSignElement{
                        std::make_shared<TrafficSignElement>(regulatory_elements_utils::extractTypeFromNationalID(
                            trafficSignId, country, benchmarkID.substr(0, 3)))};

                    for (pugi::xml_node trafficSignChildElementChild = trafficSignChildElement.first_child();
                         trafficSignChildElementChild != nullptr;
                         trafficSignChildElementChild = trafficSignChildElementChild.next_sibling()) {
                        if ((strcmp(trafficSignChildElementChild.name(), "additionalValue")) == 0) {
                            newTrafficSignElement->addAdditionalValue(
                                trafficSignChildElementChild.first_child().value());
                        }
                    }
                    temptrafficSignContainer[arrayIndex]->addTrafficSignElement(newTrafficSignElement);
                }
                if ((strcmp(trafficSignChildElement.name(), "virtual")) == 0) {
                    temptrafficSignContainer[arrayIndex]->setVirtualElement(
                        trafficSignChildElement.first_attribute().as_bool());
                }
                if ((strcmp(trafficSignChildElement.name(), "position")) == 0) {
                    if ((strcmp(trafficSignChildElement.first_child().name(), "point")) == 0) {
                        vertex newVertex{};
                        newVertex.x = trafficSignChildElement.first_child().child("x").text().as_double();
                        newVertex.y = trafficSignChildElement.first_child().child("y").text().as_double();
                        temptrafficSignContainer[arrayIndex]->setPosition(newVertex);
                    }
                }
            }
            ++arrayIndex;
        }
    }
    return temptrafficSignContainer;
}

std::vector<std::shared_ptr<TrafficLight>> CommonRoadFactory2020a::createTrafficLights() {
    std::vector<std::shared_ptr<TrafficLight>> tempLightContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of traffic lights
    size_t numTrafficLights{static_cast<size_t>(
        std::distance(commonRoad.children("trafficLight").begin(), commonRoad.children("trafficLight").end()))};
    tempLightContainer.clear();
    tempLightContainer.reserve(numTrafficLights); // Already know the size --> Faster memory allocation

    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        // get traffic lights
        if ((strcmp(roadElements.name(), "trafficLight")) == 0) {
            bool directionSet{false}; // if not direction is provided in -> direction::all
            std::shared_ptr<TrafficLight> tempTrafficLight = std::make_shared<TrafficLight>();
            tempLightContainer.emplace_back(tempTrafficLight);
            tempLightContainer[arrayIndex]->setId(roadElements.first_attribute().as_ullong());
            for (pugi::xml_node trafficLightChildElement = roadElements.first_child();
                 trafficLightChildElement != nullptr;
                 trafficLightChildElement = trafficLightChildElement.next_sibling()) {
                // get traffic light cycle
                if ((strcmp(trafficLightChildElement.name(), "cycle")) == 0) {
                    for (pugi::xml_node trafficLightCycleChildElement = trafficLightChildElement.first_child();
                         trafficLightCycleChildElement != nullptr;
                         trafficLightCycleChildElement = trafficLightCycleChildElement.next_sibling()) {
                        if ((strcmp(trafficLightCycleChildElement.name(), "cycleElement")) == 0) {
                            std::string duration = trafficLightCycleChildElement.first_child().first_child().value();
                            std::string color =
                                trafficLightCycleChildElement.first_child().next_sibling().first_child().value();
                            tempLightContainer[arrayIndex]->addCycleElement(
                                {TrafficLight::matchTrafficLightState(color), std::stoul(duration)});
                        }
                        if ((strcmp(trafficLightCycleChildElement.name(), "timeOffset")) == 0) {
                            tempLightContainer[arrayIndex]->setOffset(
                                std::stoul(trafficLightCycleChildElement.first_child().value()));
                        }
                    }
                }
                if ((strcmp(trafficLightChildElement.name(), "direction")) == 0) {
                    tempLightContainer[arrayIndex]->setDirection(
                        regulatory_elements_utils::matchDirections(trafficLightChildElement.first_child().value()));
                    directionSet = true;
                }
                if ((strcmp(trafficLightChildElement.name(), "active")) == 0) {
                    tempLightContainer[arrayIndex]->setActive(
                        strcasecmp("true", trafficLightChildElement.first_child().value()) == 0);
                }
                if ((strcmp(trafficLightChildElement.name(), "position")) == 0) {
                    if ((strcmp(trafficLightChildElement.first_child().name(), "point")) == 0) {
                        vertex newVertex{};
                        newVertex.x = trafficLightChildElement.first_child().child("x").text().as_double();
                        newVertex.y = trafficLightChildElement.first_child().child("y").text().as_double();
                        tempLightContainer[arrayIndex]->setPosition(newVertex);
                    }
                }
            }
            if (!directionSet)
                tempLightContainer[arrayIndex]->setDirection(Direction::all);
            ++arrayIndex;
        }
    }
    return tempLightContainer;
}

std::vector<std::shared_ptr<Intersection>>
CommonRoadFactory2020a::createIntersections(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempIntersectionContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // get the number of intersections
    size_t numIntersections{static_cast<size_t>(
        std::distance(commonRoad.children("intersection").begin(), commonRoad.children("intersection").end()))};
    tempIntersectionContainer.clear();
    tempIntersectionContainer.reserve(numIntersections); // Already know the size --> Faster memory allocation

    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "intersection")) == 0) {
            std::shared_ptr<Intersection> tempIntersection = std::make_shared<Intersection>();
            tempIntersectionContainer.emplace_back(tempIntersection);
            tempIntersectionContainer[arrayIndex]->setId(roadElements.first_attribute().as_ullong());
            std::map<size_t, size_t> tmpLeftOf;

            for (pugi::xml_node intersectionChildElement = roadElements.first_child();
                 intersectionChildElement != nullptr;
                 intersectionChildElement = intersectionChildElement.next_sibling()) {
                // get incoming
                if ((strcmp(intersectionChildElement.name(), "incoming")) == 0) {
                    std::shared_ptr<IncomingGroup> inc = std::make_shared<IncomingGroup>();
                    inc->setId(intersectionChildElement.first_attribute().as_ullong());
                    std::vector<std::shared_ptr<Lanelet>> incomingLanelet;
                    std::vector<std::shared_ptr<Lanelet>> outgoingsRight;
                    std::vector<std::shared_ptr<Lanelet>> outgoingsStraight;
                    std::vector<std::shared_ptr<Lanelet>> outgoingsLeft;
                    for (pugi::xml_node incomingChildElementChild = intersectionChildElement.first_child();
                         incomingChildElementChild != nullptr;
                         incomingChildElementChild = incomingChildElementChild.next_sibling()) {
                        if ((strcmp(incomingChildElementChild.name(), "incomingLanelet")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId()) {
                                    incomingLanelet.push_back(let);
                                    let->addLaneletType(LaneletType::incoming);
                                }
                            }
                            inc->setIncomingLanelets(incomingLanelet);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "outgoingRight")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId())
                                    outgoingsRight.push_back(let);
                            }
                            inc->setRightOutgoings(outgoingsRight);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "outgoingStraight")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId())
                                    outgoingsStraight.push_back(let);
                            }
                            inc->setStraightOutgoings(outgoingsStraight);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "outgoingLeft")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId())
                                    outgoingsLeft.push_back(let);
                            }
                            inc->setLeftOutgoings(outgoingsLeft);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "successorsRight")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId())
                                    outgoingsRight.push_back(let);
                            }
                            inc->setRightOutgoings(outgoingsRight);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "successorsStraight")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId())
                                    outgoingsStraight.push_back(let);
                            }
                            inc->setStraightOutgoings(outgoingsStraight);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "successorsLeft")) == 0) {
                            for (const auto &let : lanelets) {
                                if (incomingChildElementChild.attribute("ref").as_ullong() == let->getId())
                                    outgoingsLeft.push_back(let);
                            }
                            inc->setLeftOutgoings(outgoingsLeft);
                        }
                        if ((strcmp(incomingChildElementChild.name(), "isLeftOf")) == 0)
                            tmpLeftOf.insert_or_assign(inc->getId(),
                                                       incomingChildElementChild.attribute("ref").as_ullong());
                    }
                    tempIntersectionContainer[arrayIndex]->addIncomingGroup(inc);
                }
                if ((strcmp(intersectionChildElement.name(), "crossing")) == 0) {
                    for (pugi::xml_node crossingElement = intersectionChildElement.first_child();
                         crossingElement != nullptr; crossingElement = crossingElement.next_sibling())
                        for (const auto &let : lanelets)
                            if (crossingElement.attribute("ref").as_ullong() == let->getId())
                                ; // tempIntersection->addCrossing(let); TODO add to incomingGroups
                }
            }
            // iterate over all incoming lefts and assign correct reference
            for (auto const &[key, val] : tmpLeftOf) {
                for (const auto &inc1 : tempIntersectionContainer[arrayIndex]->getIncomingGroups()) {
                    if (inc1->getId() == key)
                        for (const auto &inc2 : tempIntersectionContainer[arrayIndex]->getIncomingGroups()) {
                            if (inc2->getId() == val)
                                inc1->setIsLeftOf(inc2);
                        }
                }
            }
            ++arrayIndex;
        }
    }
    return tempIntersectionContainer;
}

std::string CommonRoadFactory2020a::benchmarkID() { return doc->child("commonRoad").attribute("benchmarkID").value(); }

std::vector<std::shared_ptr<PlanningProblem>> CommonRoadFactory2020a::createPlanningProblems() {
    std::vector<std::shared_ptr<PlanningProblem>> planningProblemList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with planning problems
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "planningProblem")) == 0) {
            XMLReader::extractPlanningProblem(planningProblemList, roadElements);
        }
    }

    return planningProblemList;
}
