#include <cstring>
#include <memory>
#include <vector>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/roadNetwork/road_network.h>

#include "commonroad_factory_2018b.h"
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

double CommonRoadFactory2018b::getTimeStepSize() {
    return doc->child("commonRoad").attribute("timeStepSize").as_double();
}

std::vector<std::shared_ptr<Obstacle>> CommonRoadFactory2018b::createObstacles() {
    std::vector<std::shared_ptr<Obstacle>> obstacleList{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    // iterate over all nodes and continue working with dynamic and static obstacles
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "obstacle")) == 0) {
            if ((strcmp(roadElements.first_child().text().as_string(), "dynamic")) == 0) {
                XMLReader::createDynamicObstacle(obstacleList, roadElements);
            } else if ((strcmp(roadElements.first_child().text().as_string(), "static")) == 0) {
                XMLReader::extractStaticObstacle(obstacleList, roadElements);
            }
        }
    }
    return obstacleList;
}

std::vector<std::shared_ptr<Lanelet>>
CommonRoadFactory2018b::createLanelets(std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                       std::vector<std::shared_ptr<TrafficLight>> trafficLights) {

    std::vector<std::shared_ptr<Lanelet>> tempLaneletContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");
    XMLReader::initializeLanelets(tempLaneletContainer, commonRoad);

    // get the other values of the lanelets
    size_t arrayIndex{0};
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "lanelet")) == 0) {
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
                if ((strcmp(child.name(), "speedLimit")) == 0) {
                    // laneletId + 4000 is the id of the virtual traffic sign element.
                    size_t id = tempLaneletContainer[arrayIndex]->getId();
                    for (const auto &sign : trafficSigns) {
                        if (sign->getId() == id + 4000) {
                            tempLaneletContainer[arrayIndex]->addTrafficSign(sign);
                            break;
                        }
                    }
                    continue;
                }
            }
            tempLaneletContainer[arrayIndex]->createCenterVertices();
            tempLaneletContainer[arrayIndex]->constructOuterPolygon();
            arrayIndex++;
        }
    }
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<TrafficSign>> CommonRoadFactory2018b::createTrafficSigns() {
    // look at speed limits of lanes, and create traffic signs from the speed limits

    std::vector<std::shared_ptr<TrafficSign>> tempSignContainer{};
    pugi::xml_node commonRoad = doc->child("commonRoad");

    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {

        if ((strcmp(roadElements.name(), "lanelet")) == 0) {
            for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
                if ((strcmp(child.name(), "speedLimit")) == 0) {
                    size_t laneletId = roadElements.first_attribute().as_ullong();
                    std::string speedLimit = child.child_value();

                    std::shared_ptr<TrafficSignElement> signElem =
                        std::make_shared<TrafficSignElement>(TrafficSignTypes::MAX_SPEED);
                    signElem->setAdditionalValues(std::vector{std::move(speedLimit)});
                    std::shared_ptr<TrafficSign> sign = std::make_shared<TrafficSign>();
                    sign->setId(laneletId + 4000); // hopefully this does not cause conflicts
                    sign->setVirtualElement(true);
                    sign->addTrafficSignElement(signElem);
                    tempSignContainer.push_back(sign);
                    break;
                }
            }
        }
    }
    return tempSignContainer;
}

std::vector<std::shared_ptr<TrafficLight>> CommonRoadFactory2018b::createTrafficLights() {

    std::vector<std::shared_ptr<TrafficLight>> tempLaneletContainer{};
    return tempLaneletContainer;
}

std::vector<std::shared_ptr<Intersection>>
CommonRoadFactory2018b::createIntersections(const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    std::vector<std::shared_ptr<Intersection>> tempLaneletContainer{};
    return tempLaneletContainer;
}

std::string CommonRoadFactory2018b::benchmarkID() { return doc->child("commonRoad").attribute("benchmarkID").value(); }

std::vector<std::shared_ptr<PlanningProblem>> CommonRoadFactory2018b::createPlanningProblems() {
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
