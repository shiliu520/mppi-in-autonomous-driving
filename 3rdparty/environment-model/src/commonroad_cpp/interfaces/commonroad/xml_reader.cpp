#include <cstdint>
#include <cstring>
#include <utility>

#include <commonroad_cpp/geometry/circle.h>
#include <commonroad_cpp/geometry/polygon.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <pugixml.hpp>

#include "commonroad_cpp/auxiliaryDefs/interval.h"
#include "commonroad_cpp/goal_state.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include "commonroad_cpp/planning_problem.h"
#include "commonroad_cpp/roadNetwork/intersection/intersection.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>

#include "commonroad_factory_2018b.h"
#include "commonroad_factory_2020a.h"

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>

#include "commonroad_cpp/geometry/shape_group.h"
#include "commonroad_cpp/obstacle/occupancy.h"

double XMLReader::extractTimeStepSize(const std::string &xmlFile) {
    const auto doc = std::make_unique<pugi::xml_document>();
    if (!doc->load_file(xmlFile.c_str()))
        throw std::runtime_error("Couldn't load XML-File: " + xmlFile);
    return doc->child("commonRoad").attribute("timeStepSize").as_double();
}

std::unique_ptr<CommonRoadFactory> createCommonRoadFactory(const std::string &xmlFile) {
    auto doc = std::make_unique<pugi::xml_document>();

    if (!doc->load_file(xmlFile.c_str()))
        throw std::runtime_error("Couldn't load XML-File: " + xmlFile);

    const auto *const version = doc->child("commonRoad").attribute("commonRoadVersion").value();
    if (strcmp(version, "2017a") == 0 || strcmp(version, "2018b") == 0)
        return std::make_unique<CommonRoadFactory2018b>(std::move(doc));
    if (strcmp(version, "2020a") == 0)
        return std::make_unique<CommonRoadFactory2020a>(std::move(doc));
    throw std::runtime_error("This CommonRoad version is not supported.");
}

std::vector<std::shared_ptr<Obstacle>> XMLReader::createObstacleFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createObstacles();
}

std::vector<std::shared_ptr<Lanelet>>
XMLReader::createLaneletFromXML(const std::string &xmlFile, std::vector<std::shared_ptr<TrafficSign>> trafficSigns,
                                std::vector<std::shared_ptr<TrafficLight>> trafficLights) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createLanelets(std::move(trafficSigns), std::move(trafficLights));
}
std::shared_ptr<World> XMLReader::createWorldFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    auto obstacle = factory->createObstacles();
    std::vector<std::shared_ptr<Obstacle>> dummyEgo;
    auto cou = extractCountryFromXML(xmlFile);
    auto signs = factory->createTrafficSigns();
    auto lights = factory->createTrafficLights();
    auto lanelets = factory->createLanelets(signs, lights);
    auto inters = factory->createIntersections(lanelets);
    auto roadNetwork = std::make_shared<RoadNetwork>(lanelets, cou, signs, lights, inters);
    for (const auto &inter : roadNetwork->getIntersections())
        inter->computeMemberLanelets(roadNetwork);
    double timeStepSize{factory->getTimeStepSize()};
    return std::make_shared<World>(factory->benchmarkID(), 0, roadNetwork, dummyEgo, obstacle, timeStepSize);
}

std::vector<std::shared_ptr<TrafficSign>> XMLReader::createTrafficSignFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficSigns();
}

SupportedTrafficSignCountry XMLReader::extractCountryFromXML(const std::string &xmlFile) {
    std::vector<std::string> result;
    std::stringstream stream{xmlFile}; // create string stream from the string
    while (stream.good()) {
        std::string substr;
        getline(stream, substr, '/'); // get first string delimited by comma
        result.push_back(substr);
    }
    auto name{result.back().substr(0, 3)};
    return RoadNetwork::matchStringToCountry(name);
}

std::vector<std::shared_ptr<TrafficLight>> XMLReader::createTrafficLightFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createTrafficLights();
}

std::vector<std::shared_ptr<Intersection>>
XMLReader::createIntersectionFromXML(const std::string &xmlFile,
                                     const std::vector<std::shared_ptr<Lanelet>> &lanelets) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createIntersections(lanelets);
}

std::shared_ptr<State> XMLReader::extractInitialState(const pugi::xml_node &child) {
    const pugi::xml_node states = child;

    const auto timeStep = states.child("time").child("exact").text().as_ullong();
    const auto xPosition = states.child("position").child("point").child("x").text().as_double();
    const auto yPosition = states.child("position").child("point").child("y").text().as_double();
    const auto orientation = states.child("orientation").child("exact").text().as_double();

    if (std::isnan(xPosition) or std::isnan(yPosition) or std::isnan(orientation)) {
        throw std::runtime_error("XMLReader::extractInitialState: Value is NaN.");
    }

    State initialState;
    initialState.setTimeStep(timeStep);
    initialState.setXPosition(xPosition);
    initialState.setYPosition(yPosition);
    initialState.setGlobalOrientation(orientation);
    if (states.child("velocity").child("exact").text() != nullptr)
        if (std::isnan(states.child("velocity").child("exact").text().as_double()))
            throw std::runtime_error("XMLReader::extractInitialState: Velocity is NaN.");
    initialState.setVelocity(states.child("velocity").child("exact").text().as_double());
    if (states.child("acceleration").child("exact").text() != nullptr)
        initialState.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(initialState);
}

std::shared_ptr<SignalState> XMLReader::extractSignalState(const pugi::xml_node &child) {
    pugi::xml_node states = child;
    SignalState initialSignalState;
    initialSignalState.setTimeStep(states.child("time").child("exact").text().as_ullong());
    if (states.child("horn") != nullptr)
        initialSignalState.setHorn(states.child("horn").text().as_bool());
    if (states.child("indicatorLeft") != nullptr)
        initialSignalState.setIndicatorLeft(states.child("indicatorLeft").text().as_bool());
    if (states.child("indicatorRight") != nullptr)
        initialSignalState.setIndicatorRight(states.child("indicatorRight").text().as_bool());
    if (states.child("brakingLights") != nullptr)
        initialSignalState.setBrakingLights(states.child("brakingLights").text().as_bool());
    if (states.child("hazardWarningLights") != nullptr)
        initialSignalState.setHazardWarningLights(states.child("hazardWarningLights").text().as_bool());
    if (states.child("flashingBlueLights") != nullptr)
        initialSignalState.setFlashingBlueLights(states.child("flashingBlueLights").text().as_bool());
    return std::make_shared<SignalState>(initialSignalState);
}

std::shared_ptr<Occupancy> XMLReader::extractOccupancy(const pugi::xml_node &child) {
    Occupancy occ;
    occ.setTimeStep(child.child("time").child("exact").text().as_ullong());
    std::vector<std::shared_ptr<Shape>> shapes;
    for (pugi::xml_node shape = child.first_child(); shape != nullptr; shape = shape.next_sibling()) {
        for (pugi::xml_node xmlShape = shape.first_child(); xmlShape != nullptr; xmlShape = xmlShape.next_sibling()) {
            if ((strcmp(xmlShape.name(), "rectangle")) == 0)
                shapes.push_back(std::make_shared<Rectangle>(xmlShape.child("length").text().as_double(),
                                                             xmlShape.child("width").text().as_double()));
            else if ((strcmp(xmlShape.name(), "circle")) == 0) {
                vertex vert{xmlShape.child("center").child("x").text().as_double(),
                            xmlShape.child("center").child("y").text().as_double()};
                shapes.push_back(std::make_shared<Circle>(xmlShape.child("radius").text().as_double(), vert));
            } else if (strcmp(xmlShape.name(), "polygon") == 0) {
                std::vector<vertex> vertices;
                for (pugi::xml_node points = xmlShape.first_child(); points != nullptr;
                     points = points.next_sibling()) {
                    if ((strcmp(points.name(), "point")) == 0)
                        vertices.push_back(
                            {points.child("x").text().as_double(), points.child("y").text().as_double()});
                }
                shapes.push_back(std::make_shared<Polygon>(vertices));
            }
        }
    }
    if (shapes.size() == 1)
        occ.setShape(shapes.front());
    else
        occ.setShape(std::make_shared<ShapeGroup>(shapes));
    return std::make_shared<Occupancy>(occ);
}

std::shared_ptr<State> XMLReader::extractState(const pugi::xml_node &states) {
    const auto timeStep = states.child("time").child("exact").text().as_ullong();
    const auto xPosition = states.child("position").child("point").child("x").text().as_double();
    const auto yPosition = states.child("position").child("point").child("y").text().as_double();
    const auto orientation = states.child("orientation").child("exact").text().as_double();
    const auto velocity = states.child("velocity").child("exact").text().as_double();

    if (std::isnan(velocity) or std::isnan(xPosition) or std::isnan(yPosition) or std::isnan(orientation)) {
        throw std::runtime_error("XMLReader::extractState: Value is NaN.");
    }

    State sta;
    sta.setTimeStep(timeStep);
    sta.setXPosition(xPosition);
    sta.setYPosition(yPosition);
    sta.setGlobalOrientation(orientation);
    sta.setVelocity(velocity);
    if (states.child("acceleration").child("exact").text() != nullptr)
        sta.setAcceleration(states.child("acceleration").child("exact").text().as_double());
    return std::make_shared<State>(sta);
}

void XMLReader::extractObstacleShape(const std::shared_ptr<Obstacle> &obstacle, pugi::xml_node child) {
    std::vector<Shape> shapes;
    if ((strcmp(child.first_child().name(), "rectangle")) == 0)
        obstacle->setGeoShape(std::make_unique<Rectangle>(child.first_child().child("length").text().as_double(),
                                                          child.first_child().child("width").text().as_double()));
    else if ((strcmp(child.first_child().name(), "circle")) == 0) {
        vertex vert{child.first_child().child("center").child("x").text().as_double(),
                    child.first_child().child("center").child("y").text().as_double()};
        obstacle->setGeoShape(std::make_unique<Circle>(child.first_child().child("radius").text().as_double(), vert));
    } else if ((strcmp(child.first_child().name(), "polygon")) == 0) {
        std::vector<vertex> vertices;
        for (pugi::xml_node points = child.first_child(); points != nullptr; points = points.next_sibling()) {
            if ((strcmp(points.name(), "point")) == 0)
                vertices.push_back({points.child("x").text().as_double(), points.child("y").text().as_double()});
        }
        obstacle->setGeoShape(std::make_unique<Polygon>(vertices));
    }
}

void XMLReader::createDynamicObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();

    tempObstacle->setActuatorParameters(ActuatorParameters::vehicleDefaults());
    tempObstacle->setSensorParameters(SensorParameters::dynamicDefaults());
    tempObstacle->setObstacleRole(ObstacleRole::DYNAMIC);

    // extract ID, type, shape, initial state, and trajectory
    tempObstacle->setId(roadElements.first_attribute().as_ullong());
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "type")) == 0)
            tempObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(child.text().as_string()));
        if ((strcmp(child.name(), "shape")) == 0) {
            extractObstacleShape(tempObstacle, child);
            continue;
        }
        if ((strcmp(child.name(), "initialState")) == 0) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
        } else if ((strcmp(child.name(), "initialSignalState")) == 0) {
            std::shared_ptr<SignalState> initialSignalState{XMLReader::extractSignalState(child)};
            tempObstacle->setCurrentSignalState(initialSignalState);
        } else if ((strcmp(child.name(), "trajectory")) == 0) {
            for (pugi::xml_node states = child.first_child(); states != nullptr; states = states.next_sibling())
                tempObstacle->appendStateToTrajectoryPrediction(XMLReader::extractState(states));
        } else if ((strcmp(child.name(), "signalSeries")) == 0) {
            for (pugi::xml_node states = child.first_child(); states != nullptr; states = states.next_sibling())
                tempObstacle->appendSignalStateToSeries(XMLReader::extractSignalState(states));
        } else if (strcmp(child.name(), "occupancySet") == 0) {
            for (pugi::xml_node occ = child.first_child(); occ != nullptr; occ = occ.next_sibling()) {
                tempObstacle->appendOccupancyToSetBasedPrediction(XMLReader::extractOccupancy(occ));
            }
        }
    }

    obstacleList.emplace_back(tempObstacle);
}

void XMLReader::extractStaticObstacle(std::vector<std::shared_ptr<Obstacle>> &obstacleList,
                                      const pugi::xml_node &roadElements) {
    std::shared_ptr<Obstacle> tempObstacle = std::make_shared<Obstacle>();
    tempObstacle->setObstacleRole(ObstacleRole::STATIC);
    tempObstacle->setSensorParameters(SensorParameters::staticDefaults());

    // extract ID, type, shape, and initial state
    tempObstacle->setId(roadElements.first_attribute().as_ullong());
    tempObstacle->setIsStatic(true);
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "type")) == 0)
            tempObstacle->setObstacleType(obstacle_operations::matchStringToObstacleType(child.text().as_string()));
        if ((strcmp(child.name(), "shape")) == 0) {
            extractObstacleShape(tempObstacle, child);
            continue;
        } else if (strcmp(child.name(), "initialState") == 0) {
            std::shared_ptr<State> initialState{XMLReader::extractInitialState(child)};
            tempObstacle->setCurrentState(initialState);
        }
    }
    obstacleList.emplace_back(tempObstacle);
}

size_t XMLReader::initializeLanelets(std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                     const pugi::xml_node &commonRoad) {
    // get the number of lanelets
    size_t numLanelets{static_cast<size_t>(
        std::distance(commonRoad.children("lanelet").begin(), commonRoad.children("lanelet").end()))};
    tempLaneletContainer.clear();
    tempLaneletContainer.reserve(numLanelets); // Already know the size --> Faster memory allocation

    // all lanelets must be initialized first because they are referencing each other
    for (size_t i{0}; i < numLanelets; i++) {
        std::shared_ptr<Lanelet> tempLanelet =
            std::make_shared<Lanelet>(); // make_shared is faster than (new Lanelet());
        tempLaneletContainer.emplace_back(tempLanelet);
    }

    int arrayIndex = 0;
    // set id of lanelets
    for (pugi::xml_node roadElements = commonRoad.first_child(); roadElements != nullptr;
         roadElements = roadElements.next_sibling()) {
        if ((strcmp(roadElements.name(), "lanelet")) == 0) {
            tempLaneletContainer[static_cast<unsigned long>(arrayIndex)]->setId(
                static_cast<size_t>(roadElements.first_attribute().as_int()));
            arrayIndex++;
        }
    }
    return numLanelets;
}

void XMLReader::extractLaneletBoundary(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                       size_t arrayIndex, const pugi::xml_node &child, const char *side) {
    for (pugi::xml_node points = child.first_child(); points != nullptr; points = points.next_sibling()) {
        vertex newVertex;
        if ((strcmp(points.name(), "point")) == 0) {
            newVertex = {points.child("x").text().as_double(), points.child("y").text().as_double()};
            if ((strcmp(side, "rightBound")) == 0)
                tempLaneletContainer[arrayIndex]->addRightVertex(newVertex);
            else if ((strcmp(side, "leftBound")) == 0)
                tempLaneletContainer[arrayIndex]->addLeftVertex(newVertex);
        }
        if ((strcmp(points.name(), "lineMarking")) == 0) {
            auto lineMarking = lanelet_operations::matchStringToLineMarking(points.first_child().value());
            if ((strcmp(side, "rightBound")) == 0)
                tempLaneletContainer[arrayIndex]->setLineMarkingRight(lineMarking);
            else if ((strcmp(side, "leftBound")) == 0)
                tempLaneletContainer[arrayIndex]->setLineMarkingLeft(lineMarking);
        }
    }
}

void XMLReader::extractLaneletPreSuc(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                     size_t arrayIndex, const pugi::xml_node &child, const char *type) {
    size_t lid = static_cast<size_t>(child.first_attribute().as_ullong());
    for (size_t i{0}; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == lid) {
            if ((strcmp(type, "successor")) == 0)
                tempLaneletContainer[arrayIndex]->addSuccessor(tempLaneletContainer[i]);
            else if ((strcmp(type, "predecessor")) == 0)
                tempLaneletContainer[arrayIndex]->addPredecessor(tempLaneletContainer[i]);
            break;
        }
    }
}

void XMLReader::extractLaneletAdjacency(const std::vector<std::shared_ptr<Lanelet>> &tempLaneletContainer,
                                        size_t arrayIndex, const pugi::xml_node &child, const char *type) {
    size_t adjacentId = static_cast<size_t>(child.attribute("ref").as_ullong());
    bool oppositeDir{false};
    if ((strcmp(child.attribute("drivingDir").as_string(), "same")) == 0)
        oppositeDir = false;
    else if ((strcmp(child.attribute("drivingDir").as_string(), "opposite")) == 0)
        oppositeDir = true;
    for (size_t i{0}; i < tempLaneletContainer.size(); i++) {
        if (tempLaneletContainer[i]->getId() == adjacentId) {
            if ((strcmp(type, "adjacentLeft")) == 0)
                tempLaneletContainer[arrayIndex]->setLeftAdjacent(tempLaneletContainer[i], oppositeDir);
            else if ((strcmp(type, "adjacentRight")) == 0)
                tempLaneletContainer[arrayIndex]->setRightAdjacent(tempLaneletContainer[i], oppositeDir);
            break;
        }
    }
}

// TODO: Merge with extractInitialState
static std::shared_ptr<InitialState> extractInitialStateForPlanningProblem(const pugi::xml_node &child) {
    const pugi::xml_node states = child;

    auto timeStep = states.child("time").child("exact").text().as_ullong();
    auto xPosition = states.child("position").child("point").child("x").text().as_double();
    auto yPosition = states.child("position").child("point").child("y").text().as_double();
    auto orientation = states.child("orientation").child("exact").text().as_double();
    auto velocity = states.child("velocity").child("exact").text().as_double();
    auto yawRate = states.child("yawRate").child("exact").text().as_double();
    auto slipAngle = states.child("slipAngle").child("exact").text().as_double();

    if (std::isnan(velocity) or std::isnan(yawRate) or std::isnan(slipAngle) or std::isnan(xPosition) or
        std::isnan(yPosition) or std::isnan(orientation)) {
        throw std::runtime_error("extractInitialStateForPlanningProblem: Value is NaN.");
    }

    std::optional<double> acceleration;
    if (states.child("acceleration").child("exact").text() != nullptr) {
        acceleration = states.child("acceleration").child("exact").text().as_double();
    }

    // FIXME: InitialState should use optional for acceleration
    auto acceleration_value = acceleration.value_or(0.0);

    return std::make_shared<InitialState>(timeStep, xPosition, yPosition, orientation, velocity, acceleration_value,
                                          yawRate, slipAngle);
}

void XMLReader::extractPlanningProblem(std::vector<std::shared_ptr<PlanningProblem>> &planningProblemList,
                                       const pugi::xml_node &roadElements) {
    auto id = roadElements.first_attribute().as_ullong();

    auto initial_state = extractInitialStateForPlanningProblem(roadElements.child("initialState"));

    std::vector<GoalState> goal_states;
    for (pugi::xml_node child = roadElements.first_child(); child != nullptr; child = child.next_sibling()) {
        if ((strcmp(child.name(), "goalState")) != 0) {
            continue;
        }

        auto time_node = child.child("time");
        if (time_node == nullptr) {
            throw std::runtime_error{"goal state needs to have a time interval"};
        }
        auto time = std::make_pair(time_node.child("intervalStart").text().as_ullong(),
                                   time_node.child("intervalEnd").text().as_ullong());

        auto velocity_node = child.child("velocity");
        std::optional<FloatInterval> velocity;
        if (velocity_node != nullptr) {
            auto start_node = velocity_node.child("intervalStart");
            auto end_node = velocity_node.child("intervalEnd");
            if (start_node == nullptr || end_node == nullptr) {
                throw std::runtime_error{"invalid interval node"};
            }
            velocity = std::make_pair(start_node.text().as_double(), end_node.text().as_double());
        }

        auto orientation_node = child.child("orientation");
        std::optional<FloatInterval> orientation;
        if (orientation_node != nullptr) {
            auto start_node = orientation_node.child("intervalStart");
            auto end_node = orientation_node.child("intervalEnd");
            if (start_node == nullptr || end_node == nullptr) {
                throw std::runtime_error{"invalid interval node"};
            }
            orientation = std::make_pair(start_node.text().as_double(), end_node.text().as_double());
        }

        auto position_node = child.child("position");
        // TODO: Support other goal position types
        std::vector<GoalPosition> goal_positions;
        if (position_node != nullptr) {
            for (pugi::xml_node position_ref = roadElements.first_child(); position_ref != nullptr;
                 position_ref = position_ref.next_sibling()) {
                if (strcmp(position_ref.name(), "lanelet") == 0) {
                    auto lanelet_id = static_cast<uint32_t>(position_ref.attribute("ref").as_ullong());
                    goal_positions.emplace_back(lanelet_id);
                } else if (strcmp(position_ref.name(), "rectangle") == 0) {
                    std::shared_ptr<Shape> shape =
                        std::make_shared<Rectangle>(position_ref.child("length").text().as_double(),
                                                    position_ref.child("width").text().as_double());
                    goal_positions.emplace_back(shape);
                } else if (strcmp(position_ref.name(), "circle") == 0) {
                    std::shared_ptr<Shape> shape =
                        std::make_shared<Circle>(position_ref.child("radius").text().as_double(),
                                                 vertex{
                                                     position_ref.child("center").child("x").text().as_double(),
                                                     position_ref.child("center").child("y").text().as_double(),
                                                 });
                    goal_positions.emplace_back(shape);
                }
            }
        }

        goal_states.emplace_back(time, velocity, orientation, goal_positions);
    }

    auto planningProblem = std::make_shared<PlanningProblem>(id, initial_state, goal_states);
    planningProblemList.push_back(planningProblem);
}

std::vector<std::shared_ptr<PlanningProblem>> XMLReader::createPlanningProblemFromXML(const std::string &xmlFile) {
    const auto factory = createCommonRoadFactory(xmlFile);
    return factory->createPlanningProblems();
}
