#include "commonroad_cpp/geometry/circle.h"

#include <cstddef>
#include <geometry/curvilinear_coordinate_system.h>

#include <algorithm> // for max, min
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <cmath>
#include <stdexcept> // for logic_error
#include <string>    // for operator+
#include <utility>

#include <Eigen/Core> // for Vector2d

#include <boost/geometry/geometries/ring.hpp> // for ring

#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/geometry/rectangle.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/obstacle/obstacle_reference.h>
#include <commonroad_cpp/obstacle/occupancy.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <spdlog/spdlog.h>

#include "commonroad_cpp/geometry/polygon.h"
#include "commonroad_cpp/geometry/shape_group.h"

static std::array<double, 4> rotatedCornerLatitudes(const Rectangle &rect, const double theta) {
    const double width = rect.getWidth();
    const double length = rect.getLength();

    return {width / 2 * cos(theta) - length / 2 * sin(theta), width / 2 * cos(theta) - -length / 2 * sin(theta),
            -width / 2 * cos(theta) - length / 2 * sin(theta), -width / 2 * cos(theta) - -length / 2 * sin(theta)};
}

static std::array<double, 4> rotatedCornerLongitudes(const Rectangle &rect, const double theta) {
    const double width = rect.getWidth();
    const double length = rect.getLength();

    return {length / 2 * cos(theta) - width / 2 * sin(theta), length / 2 * cos(theta) - -width / 2 * sin(theta),
            -length / 2 * cos(theta) - width / 2 * sin(theta), -length / 2 * cos(theta) - -width / 2 * sin(theta)};
}

static double rotatedMinimumLatitude(const Rectangle &rect, const double theta) {
    const auto latitudes = rotatedCornerLatitudes(rect, theta);
    return *std::min_element(latitudes.cbegin(), latitudes.cend());
}

static double rotatedMaximumLatitude(const Rectangle &rect, const double theta) {
    const auto latitudes = rotatedCornerLatitudes(rect, theta);
    return *std::max_element(latitudes.cbegin(), latitudes.cend());
}

static double rotatedMinimumLongitude(const Rectangle &rect, const double theta) {
    const auto longitudes = rotatedCornerLongitudes(rect, theta);
    return *std::min_element(longitudes.cbegin(), longitudes.cend());
}

static double rotatedMaximumLongitude(const Rectangle &rect, const double theta) {
    const auto longitudes = rotatedCornerLongitudes(rect, theta);
    return *std::max_element(longitudes.cbegin(), longitudes.cend());
}

Obstacle::Obstacle(const size_t obstacleId, const ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
                   const ObstacleType obstacleType, const double vMax, const double aMax, const double aMaxLong,
                   const double aMinLong, const double reactionTime, state_map_t trajectoryPrediction, double length,
                   double width)
    : Obstacle{obstacleId,
               obstacleRole,
               std::move(currentState),
               obstacleType,
               ActuatorParameters{vMax, aMax, aMaxLong, aMinLong, aMinLong},
               SensorParameters{250.0, 250.0},
               TimeParameters{TimeParameters::dynamicDefaults().getRelevantHistorySize(), reactionTime,
                              TimeParameters::dynamicDefaults().getTimeStepSize()},
               std::move(trajectoryPrediction),
               std::make_unique<Rectangle>(length, width)} {}

Obstacle::Obstacle(const size_t obstacleId, const ObstacleRole obstacleRole, std::shared_ptr<State> currentState,
                   const ObstacleType obstacleType, const ActuatorParameters &actuatorParameters,
                   SensorParameters sensorParameters, const TimeParameters &timeParameters,
                   state_map_t trajectoryPrediction, std::unique_ptr<Shape> shape)
    : obstacleId(obstacleId), obstacleRole(obstacleRole), obstacleType(obstacleType),
      actuatorParameters(actuatorParameters), sensorParameters(std::move(sensorParameters)),
      timeParameters(timeParameters), geoShape(std::move(shape)) {
    this->trajectoryPrediction.trajectoryPrediction = std::move(trajectoryPrediction);
    recordedStates.currentState = std::move(currentState);
    if (obstacleRole == ObstacleRole::STATIC)
        setIsStatic(true);

    setFirstLastTimeStep();
}

void Obstacle::setId(const size_t oId) { obstacleId = oId; }

void Obstacle::setIsStatic(const bool staticObstacle) {
    if (staticObstacle) {
        actuatorParameters = ActuatorParameters::staticDefaults();
    }
}

void Obstacle::setCurrentState(const std::shared_ptr<State> &currentState) {
    recordedStates.currentState = currentState;
    setFirstLastTimeStep();
    recordedStates.removeTimeStepFromMappingVariables(currentState->getTimeStep(), dynamicRef);
}

void Obstacle::updateHistory() {
    if (timeParameters.getRelevantHistorySize() > 0)
        recordedStates.trajectoryHistory[recordedStates.currentState->getTimeStep()] = recordedStates.currentState;
    if (recordedStates.trajectoryHistory.size() > timeParameters.getRelevantHistorySize()) {
        recordedStates.trajectoryHistory.erase(firstTimeStep);
        recordedStates.removeTimeStepFromMappingVariables(firstTimeStep, dynamicRef);
    }
}

void Obstacle::updateCurrentState(const std::shared_ptr<State> &newState) {
    if (timeParameters.getRelevantHistorySize() == 0)
        setCurrentState(newState);
    else {
        updateHistory();
        recordedStates.currentState = newState;
        setFirstLastTimeStep();
    }
}

void Obstacle::setFov(const std::vector<vertex> &fovVertices) { sensorParameters.setFov(fovVertices); }

void Obstacle::setObstacleType(const ObstacleType type) { obstacleType = type; }

void Obstacle::setActuatorParameters(const ActuatorParameters &params) { actuatorParameters = params; }

void Obstacle::setSensorParameters(SensorParameters params) { sensorParameters = std::move(params); }

void Obstacle::setTimeParameters(TimeParameters params) { timeParameters = params; }

void Obstacle::setRoadNetworkParameters(RoadNetworkParameters params) { roadNetworkParameters = params; };

void Obstacle::setTrajectoryPrediction(const state_map_t &trajPrediction) {
    trajectoryPrediction.clearCache();
    trajectoryPrediction.trajectoryPrediction = trajPrediction;
    setFirstLastTimeStep();
}

void Obstacle::setTrajectoryHistory(const state_map_t &trajHistory) {
    recordedStates.clearCache();
    recordedStates.trajectoryHistory = trajHistory;
    setFirstLastTimeStep();
}

void Obstacle::setGeoShape(std::unique_ptr<Shape> shape) { geoShape = std::move(shape); }

void Obstacle::appendStateToTrajectoryPrediction(const std::shared_ptr<State> &state) {
    trajectoryPrediction.trajectoryPrediction.insert({state->getTimeStep(), state});
    if (state->getTimeStep() > finalTimeStep)
        finalTimeStep = state->getTimeStep();
}

void Obstacle::appendOccupancyToSetBasedPrediction(const std::shared_ptr<Occupancy> &occ) {
    setBasedPrediction.setBasedPrediction.insert({occ->getTimeStep(), occ});
    if (occ->getTimeStep() > finalTimeStep)
        finalTimeStep = occ->getTimeStep();
}

void Obstacle::appendStateToHistory(const std::shared_ptr<State> &state) {
    recordedStates.trajectoryHistory.insert({state->getTimeStep(), state});
    if (state->getTimeStep() < firstTimeStep)
        firstTimeStep = state->getTimeStep();
}

size_t Obstacle::getId() const { return obstacleId; }

const std::shared_ptr<State> &Obstacle::getCurrentState() const { return recordedStates.currentState; }

bool Obstacle::timeStepExists(const size_t timeStep) const {
    // for computational reasons we assume an obstacle trajectory has equidistant time steps from the initial/current
    // state or first history state until the last trajectory prediction state
    return isStatic() or (firstTimeStep <= timeStep and timeStep <= finalTimeStep);
}

std::shared_ptr<State> Obstacle::getStateByTimeStep(const size_t timeStep) const {
    if (isStatic())
        return recordedStates.currentState;
    if (trajectoryPrediction.trajectoryPrediction.count(timeStep) == 1)
        return trajectoryPrediction.trajectoryPrediction.at(timeStep);
    if (recordedStates.currentState->getTimeStep() == timeStep)
        return recordedStates.currentState;
    if (recordedStates.trajectoryHistory.count(timeStep) == 1)
        return recordedStates.trajectoryHistory.at(timeStep);
    throw std::logic_error("Time step does not exist. Obstacle ID: " + std::to_string(this->getId()) +
                           " - Time step: " + std::to_string(timeStep) + " - First time step: " +
                           std::to_string(firstTimeStep) + " - Final time step: " + std::to_string(finalTimeStep));
}

std::shared_ptr<SignalState> Obstacle::getSignalStateByTimeStep(size_t timeStep) const {
    if (isStatic())
        return recordedStates.currentSignalState;
    if (trajectoryPrediction.signalSeries.count(timeStep) == 1)
        return trajectoryPrediction.signalSeries.at(timeStep);
    if (recordedStates.currentSignalState && recordedStates.currentSignalState->getTimeStep() == timeStep)
        return recordedStates.currentSignalState;
    if (recordedStates.signalSeriesHistory.count(timeStep) == 1)
        return recordedStates.signalSeriesHistory.at(timeStep);
    spdlog::info("Obstacle::getSignalStateByTimeStep: No signal state found. Returning default signal state.");
    return std::make_shared<SignalState>(false, false, false, false, false, false, timeStep);
}

ObstacleType Obstacle::getObstacleType() const { return obstacleType; }

ObstacleRole Obstacle::getObstacleRole() const { return obstacleRole; }

ActuatorParameters Obstacle::getActuatorParameters() const { return actuatorParameters; }

TimeParameters Obstacle::getTimeParameters() const { return timeParameters; }

SensorParameters Obstacle::getSensorParameters() const { return sensorParameters; }

RoadNetworkParameters Obstacle::getRoadNetworkParameters() const { return roadNetworkParameters; }

double Obstacle::getVmax() const { return actuatorParameters.getVmax(); }

double Obstacle::getAmax() const { return actuatorParameters.getAmax(); }

double Obstacle::getAmaxLong() const { return actuatorParameters.getAmaxLong(); }

double Obstacle::getAminLong() const { return actuatorParameters.getAminLong(); }

double Obstacle::getReactionTime() const { return timeParameters.getReactionTime(); }

std::vector<size_t> Obstacle::getPredictionTimeSteps() const {
    std::vector<size_t> timeSteps;
    boost::copy(trajectoryPrediction.trajectoryPrediction | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getHistoryTimeSteps() const {
    std::vector<size_t> timeSteps;
    boost::copy(recordedStates.trajectoryHistory | boost::adaptors::map_keys, std::back_inserter(timeSteps));
    return timeSteps;
}

std::vector<size_t> Obstacle::getTimeSteps() const {
    std::vector<size_t> timeSteps{getPredictionTimeSteps()};
    if (recordedStates.currentState == nullptr) {
        // TODO: Should we report an error in this case ?
        return std::vector<size_t>{};
    }
    if (timeSteps.empty())
        timeSteps.insert(timeSteps.begin(), recordedStates.currentState->getTimeStep());
    else {
        if (timeSteps.front() != recordedStates.currentState->getTimeStep())
            timeSteps.insert(timeSteps.begin(), recordedStates.currentState->getTimeStep());
    }
    return timeSteps;
}

state_map_t Obstacle::getTrajectoryPrediction() const { return trajectoryPrediction.trajectoryPrediction; }

state_map_t Obstacle::getTrajectoryHistory() const { return recordedStates.trajectoryHistory; }

size_t Obstacle::getTrajectoryLength() const { return trajectoryPrediction.trajectoryPrediction.size(); }

multi_polygon_type Obstacle::getOccupancyPolygonShape(const size_t timeStep) {
    return setOccupancyPolygonShape(timeStep);
}

time_step_map_t<multi_polygon_type> &Obstacle::getOccupancyPolygonShapeCache(const size_t timeStep,
                                                                             const bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.shapeAtTimeStep;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.shapeAtTimeStep;
    return trajectoryPrediction.obstacleCache.shapeAtTimeStep;
}

multi_polygon_type Obstacle::setOccupancyPolygonShape(const size_t timeStep, const bool setBased) {
    auto &shapeAtTimeStep{getOccupancyPolygonShapeCache(timeStep, setBased)};
    if (shapeAtTimeStep.count(timeStep) == 1)
        return shapeAtTimeStep[timeStep];

    if (timeStep > recordedStates.currentState->getTimeStep() and
        setBasedPrediction.setBasedPrediction.count(timeStep) == 1) {
        multi_polygon_type polygonShape{setBasedPrediction.setBasedPrediction.at(timeStep)->getOccupancyPolygonShape()};

        shapeAtTimeStep[timeStep] = polygonShape;
        return polygonShape;
    }

    multi_polygon_type polygonShape{polygon_type{}};
    const auto state{this->getStateByTimeStep(timeStep)};
    std::vector<vertex> boundingVertices;
    if (this->getGeoShape().getType() == ShapeType::rectangle) {
        boundingVertices = geometric_operations::addObjectDimensionsRectangle(
            std::vector{vertex{0.0, 0.0}}, this->getGeoShape().getLength(), this->getGeoShape().getWidth());
    } else if (this->getGeoShape().getType() == ShapeType::circle) {
        boundingVertices =
            geometric_operations::addObjectDimensionsCircle(vertex{0.0, 0.0}, this->getGeoShape().getRadius());
    } else {
        throw std::runtime_error{"obstacle shapes other than rectangle not supported by setOccupancyPolygonShape"};
    }

    /*
     * rotate and translate the vertices of the occupancy set in local
     * coordinates to the object's reference position and rotation
     */
    const std::vector<vertex> adjustedBoundingVertices = geometric_operations::rotateAndTranslateVertices(
        boundingVertices, vertex{state->getXPosition(), state->getYPosition()}, state->getGlobalOrientation());

    polygonShape.at(0).outer().resize(adjustedBoundingVertices.size() + 1);

    // make polygon shape from previously created vertices
    for (size_t i{0}; i < adjustedBoundingVertices.size(); i++) {
        polygonShape.at(0).outer()[i] = point_type{adjustedBoundingVertices[i].x, adjustedBoundingVertices[i].y};
    }

    // add first point once again at the end
    if (!adjustedBoundingVertices.empty()) {
        polygonShape.at(0).outer().back() = point_type{adjustedBoundingVertices[0].x, adjustedBoundingVertices[0].y};
    }
    shapeAtTimeStep[timeStep] = {polygonShape};
    return polygonShape;
}

Shape &Obstacle::getGeoShape() const { return *geoShape; }

std::unique_ptr<Shape> Obstacle::getShapePtr() const {
    if (geoShape->getType() == ShapeType::rectangle)
        return std::make_unique<Rectangle>(dynamic_cast<const Rectangle &>(*geoShape));
    if (geoShape->getType() == ShapeType::circle)
        return std::make_unique<Circle>(dynamic_cast<const Circle &>(*geoShape));
    if (geoShape->getType() == ShapeType::polygon)
        return std::make_unique<Polygon>(dynamic_cast<const Polygon &>(*geoShape));
    if (geoShape->getType() == ShapeType::shapeGroup)
        return std::make_unique<ShapeGroup>(dynamic_cast<const ShapeGroup &>(*geoShape));
    return nullptr;
}

std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
Obstacle::getOccupiedLaneletsCache(const size_t timeStep, const bool setBased) {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLanelets;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLanelets;
    return trajectoryPrediction.obstacleCache.occupiedLanelets;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                     const bool setBased) {
    auto &occupiedLanelets{getOccupiedLaneletsCache(timeStep, setBased)};
    if (occupiedLanelets.find(timeStep) != occupiedLanelets.end())
        return occupiedLanelets.at(timeStep);
    const multi_polygon_type polygonShape{getOccupancyPolygonShape(timeStep)};
    auto occupied{roadNetwork->findOccupiedLaneletsByShape(polygonShape)};
    occupiedLanelets.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occupied));
    return occupied;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep,
                                     const bool setBased) {
    return setOccupiedLaneletsByShape(roadNetwork, timeStep, setBased);
}

std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
Obstacle::getOccupiedLaneletsStateCache(const size_t timeStep, const bool setBased) {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLaneletsState;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLaneletsState;
    return trajectoryPrediction.obstacleCache.occupiedLaneletsState;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByState(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                     const bool setBased) {
    auto &occupiedLaneletsState{getOccupiedLaneletsStateCache(timeStep, setBased)};
    if (occupiedLaneletsState.find(timeStep) != occupiedLaneletsState.end())
        return occupiedLaneletsState.at(timeStep);
    std::vector<std::shared_ptr<Lanelet>> occ{roadNetwork->findLaneletsByPosition(
        getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition())};
    occupiedLaneletsState.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occ));
    return occ;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByFront(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                     const bool setBased) {
    auto &occupiedLaneletsFront{getOccupiedLaneletsFrontCache(timeStep, setBased)};
    if (occupiedLaneletsFront.find(timeStep) != occupiedLaneletsFront.end())
        return occupiedLaneletsFront.at(timeStep);
    const std::vector<double> front = getFrontXYCoordinates(timeStep, setBased);
    auto occ{roadNetwork->findLaneletsByPosition(front[0], front[1])};
    occupiedLaneletsFront.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occ));
    return occ;
}

std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
Obstacle::getOccupiedLaneletsFrontCache(const size_t timeStep, const bool setBased) {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLaneletsFront;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLaneletsFront;
    return trajectoryPrediction.obstacleCache.occupiedLaneletsFront;
}

std::unordered_map<time_step_t, std::vector<std::shared_ptr<Lanelet>>> &
Obstacle::getOccupiedLaneletsBackCache(const size_t timeStep, const bool setBased) {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLaneletsBack;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLaneletsBack;
    return trajectoryPrediction.obstacleCache.occupiedLaneletsBack;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsByBack(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep,
                                    const bool setBased) {
    auto &occupiedLaneletsBack{getOccupiedLaneletsBackCache(timeStep, setBased)};
    if (occupiedLaneletsBack.find(timeStep) != occupiedLaneletsBack.end())
        return occupiedLaneletsBack.at(timeStep);
    const auto back{getBackXYCoordinates(timeStep, setBased)};
    auto occ{roadNetwork->findLaneletsByPosition(back[0], back[1])};
    occupiedLaneletsBack.insert(std::pair<int, std::vector<std::shared_ptr<Lanelet>>>(timeStep, occ));
    return occ;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                     const size_t timeStep, const bool setBased) {
    return setOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep, setBased);
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsNotDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        const size_t timeStep, const bool setBased) {
    return setOccupiedLaneletsNotDrivingDirectionByShape(roadNetwork, timeStep, setBased);
}

void Obstacle::convertPointToCurvilinear(const size_t timeStep,
                                         const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                         const bool setBased) const {
    const auto state{getStateByTimeStep(timeStep)};
    Eigen::Vector2d convertedPoint{ccs->convertToCurvilinearCoords(state->getXPosition(), state->getYPosition())};
    auto ccsTangent{ccs->tangent(convertedPoint.x())};
    const double ccsOrientation = atan2(ccsTangent.y(), ccsTangent.x());
    const double theta = geometric_operations::subtractOrientations(state->getGlobalOrientation(), ccsOrientation);

    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    convertedPositions[timeStep][ccs] = {
        convertedPoint.x() - RoadNetworkParameters::numAdditionalSegmentsCCS * ccs->eps2(), convertedPoint.y(), theta};
}

std::string Obstacle::ccsErrorMsg(const size_t timeStep,
                                  const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                  const std::string &func) const {
    std::string refInfo;
    for (const auto &ref : ccs->referencePath())
        refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}; ";
    return "Obstacle::" + func +
           "Custom CCS - Curvilinear Projection Error - Obstacle ID: " + std::to_string(obstacleId) +
           " - Time Step: " + std::to_string(timeStep) + " - Reference Lane: " + refInfo +
           " - x-position: " + std::to_string(getStateByTimeStep(timeStep)->getXPosition()) +
           " - y-position: " + std::to_string(getStateByTimeStep(timeStep)->getYPosition());
}

double Obstacle::frontS(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep) {
    const double lonPosition = getLonPosition(roadNetwork, timeStep);
    const double theta = getCurvilinearOrientation(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use maximum of all corners
    return lonPosition + rotatedMaximumLongitude(rect, theta);
}

double Obstacle::frontS(const size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                        const bool setBased) {
    if (setBased and timeStep > recordedStates.currentState->getTimeStep()) {
        double frontS{0.0};
        std::vector<vertex> vertices;
        if (setBasedPrediction.setBasedPrediction.at(timeStep)->getShape()->getType() == ShapeType::polygon)
            vertices = dynamic_cast<const Polygon &>(*setBasedPrediction.setBasedPrediction.at(timeStep)->getShape())
                           .getPolygonVertices();
        else if (setBasedPrediction.setBasedPrediction.at(timeStep)->getShape()->getType() == ShapeType::shapeGroup) {
            for (const auto &shape :
                 dynamic_cast<ShapeGroup &>(*setBasedPrediction.setBasedPrediction.at(timeStep)->getShape())
                     .getShapes())
                if (shape->getType() == ShapeType::polygon)
                    for (vertex vert : dynamic_cast<const Polygon &>(*shape).getPolygonVertices())
                        vertices.push_back(vert);
        } else
            throw std::runtime_error("Obstacle::frontS: Only polygon shapes are supported for set-based predictions.");

        for (const auto &vert : vertices) {
            auto convertedPoint{ccs->convertToCurvilinearCoords(vert.x, vert.y)};
            frontS =
                std::max(frontS, convertedPoint.x() - RoadNetworkParameters::numAdditionalSegmentsCCS * ccs->eps2());
        }
        return frontS;
    }

    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "frontS"));
        }
    }
    const double lonPosition = convertedPositions[timeStep][ccs][0];
    const double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use maximum of all corners
    return lonPosition + rotatedMaximumLongitude(rect, theta);
}

double Obstacle::rearS(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep) {
    const double lonPosition = getLonPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    const double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    // use minimum of all corners
    return lonPosition + rotatedMinimumLongitude(rect, theta);
}

double Obstacle::rearS(const size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                       const bool setBased) {
    double rearS{std::numeric_limits<double>::max()};
    if (setBased and !setBasedPrediction.setBasedPrediction.empty() and timeStep > getCurrentState()->getTimeStep()) {
        std::vector<vertex> vertices;
        if (setBasedPrediction.setBasedPrediction.at(timeStep)->getShape()->getType() == ShapeType::polygon)
            vertices = dynamic_cast<const Polygon &>(*setBasedPrediction.setBasedPrediction.at(timeStep)->getShape())
                           .getPolygonVertices();
        else if (setBasedPrediction.setBasedPrediction.at(timeStep)->getShape()->getType() == ShapeType::shapeGroup) {
            for (const auto &shape :
                 dynamic_cast<ShapeGroup &>(*setBasedPrediction.setBasedPrediction.at(timeStep)->getShape())
                     .getShapes())
                if (shape->getType() == ShapeType::polygon)
                    for (vertex vert : dynamic_cast<const Polygon &>(*shape).getPolygonVertices())
                        vertices.push_back(vert);
        } else
            throw std::runtime_error("Obstacle::rearS: Only polygon shapes are supported for set-based predictions.");

        for (const auto &vert : vertices) {
            auto convertedPoint{ccs->convertToCurvilinearCoords(vert.x, vert.y)};
            rearS = std::min(rearS, convertedPoint.x() - RoadNetworkParameters::numAdditionalSegmentsCCS * ccs->eps2());
        }
        return rearS;
    }

    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "rearS"));
        }
    }
    const double lonPosition = convertedPositions[timeStep][ccs][0];
    const double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    // use minimum of all corners
    return lonPosition + rotatedMinimumLongitude(rect, theta);
}

double Obstacle::rightD(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep, const bool setBased) {
    auto &rightLatPosition{getRightLatPositionCache(timeStep, setBased)};
    const double latPos = getLatPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    const double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    rightLatPosition[timeStep] = latPos + rotatedMinimumLatitude(rect, theta);
    return rightLatPosition[timeStep];
}

double Obstacle::rightD(const size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                        const bool setBased) const {
    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "rightD"));
        }
    }
    const double latPosition = convertedPositions[timeStep][ccs][1];
    const double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    return latPosition + rotatedMinimumLatitude(rect, theta);
}

time_step_map_t<double> &Obstacle::getRightLatPositionCache(const size_t timeStep, const bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.rightLatPosition;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.rightLatPosition;
    return trajectoryPrediction.obstacleCache.rightLatPosition;
}

time_step_map_t<double> &Obstacle::getLeftLatPositionCache(const size_t timeStep, const bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.leftLatPosition;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.leftLatPosition;
    return trajectoryPrediction.obstacleCache.leftLatPosition;
}

double Obstacle::leftD(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep, const bool setBased) {
    auto &leftLatPosition{getLeftLatPositionCache(timeStep, setBased)};
    const double latPos = getLatPosition(roadNetwork, timeStep);
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);
    const double theta = getCurvilinearOrientation(roadNetwork, timeStep);

    leftLatPosition[timeStep] = latPos + rotatedMaximumLatitude(rect, theta);
    return leftLatPosition[timeStep];
}

double Obstacle::leftD(const size_t timeStep, const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                       const bool setBased) const {
    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "leftD"));
        }
    }
    const double latPosition = convertedPositions[timeStep][ccs][1];
    const double theta = convertedPositions[timeStep][ccs][2];
    const auto &rect = dynamic_cast<const Rectangle &>(*geoShape);

    return latPosition + rotatedMaximumLatitude(rect, theta);
}

double Obstacle::getLonPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().lonPosition)
        return getStateByTimeStep(timeStep)->getLonPosition();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getLonPosition();
}

double Obstacle::getLatPosition(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().latPosition)
        return getStateByTimeStep(timeStep)->getLatPosition();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getLatPosition();
}

double Obstacle::getLonPosition(const size_t timeStep,
                                const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                const bool setBased) const {
    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "getLonPosition"));
        }
    }
    return convertedPositions[timeStep][ccs][0];
}

double Obstacle::getLatPosition(const size_t timeStep,
                                const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                const bool setBased) const {
    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            std::string refInfo;
            for (const auto &ref : ccs->referencePath())
                refInfo += "{" + std::to_string(ref.x()) + ", " + std::to_string(ref.y()) + "}; ";
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "getLatPosition"));
        }
    }
    return convertedPositions[timeStep][ccs][1];
}

double Obstacle::getCurvilinearOrientation(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    if (getStateByTimeStep(timeStep)->getValidStates().curvilinearOrientation)
        return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
    convertPointToCurvilinear(roadNetwork, timeStep);
    return getStateByTimeStep(timeStep)->getCurvilinearOrientation();
}

double Obstacle::getCurvilinearOrientation(size_t timeStep,
                                           const std::shared_ptr<geometry::CurvilinearCoordinateSystem> &ccs,
                                           const bool setBased) {
    auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
    if (convertedPositions.count(timeStep) != 1 || convertedPositions[timeStep].count(ccs) != 1) {
        try {
            convertPointToCurvilinear(timeStep, ccs);
        } catch (...) {
            throw std::runtime_error(ccsErrorMsg(timeStep, ccs, "getCurvilinearOrientation"));
        }
    }
    return geometric_operations::constrainAngle(convertedPositions[timeStep][ccs][2]);
}

size_t Obstacle::getFirstTrajectoryTimeStep() const {
    // NOTE: trajectoryPrediction.trajectoryPrediction.is an unordered_map, so the first element (.begin()) is not
    // necessarily the minimum element!
    if (!trajectoryPrediction.trajectoryPrediction.empty())
        return std::min_element(trajectoryPrediction.trajectoryPrediction.begin(),
                                trajectoryPrediction.trajectoryPrediction.end())
            ->first;
    throw std::runtime_error("Obstacle::getFirstTrajectoryTimeStep: Obstacle with ID " + std::to_string(obstacleId) +
                             " has not trajectoryPrediction");
}

std::shared_ptr<Lane> Obstacle::getReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                 const size_t timeStep) {
    if (dynamicRef)
        return setReferenceLane(roadNetwork, timeStep);
    return recordedStates.occupancyRecorded.referenceLane.at(0);
}

time_step_map_t<std::shared_ptr<Lane>> &Obstacle::getReferenceLaneCache(size_t timeStep, const bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.referenceLane;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.referenceLane;
    return trajectoryPrediction.obstacleCache.referenceLane;
}

std::shared_ptr<Lane> Obstacle::setReferenceLane(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                 const size_t timeStep) {
    auto &referenceLane{getReferenceLaneCache(timeStep, false)};
    if (referenceLane.count(timeStep) == 1 and referenceLane.at(timeStep) != nullptr)
        return referenceLane.at(timeStep);

    if (const auto refLaneTmp{obstacle_reference::computeRef(*this, roadNetwork, timeStep)}; !refLaneTmp.empty())
        referenceLane[timeStep] = refLaneTmp.at(0);

    if (referenceLane.count(timeStep) == 0 or referenceLane.at(timeStep) == nullptr)
        throw std::runtime_error("Obstacle::setReferenceLane: No matching referenceLane found! Obstacle ID " +
                                 std::to_string(getId()) + " at time step " + std::to_string(timeStep));
    return referenceLane.at(timeStep);
}

time_step_map_t<ObstacleCache::curvilinear_position_map_t> &
Obstacle::convertedPositionsCache(const size_t timeStep, const bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.convertedPositions;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.convertedPositions;
    return trajectoryPrediction.obstacleCache.convertedPositions;
}

void Obstacle::convertPointToCurvilinear(const std::shared_ptr<RoadNetwork> &roadNetwork, const size_t timeStep,
                                         const bool setBased) {
    auto curRefLaneCCS{getReferenceLane(roadNetwork, timeStep)->getCurvilinearCoordinateSystem()};
    try {
        convertPointToCurvilinear(timeStep, curRefLaneCCS);
        auto &convertedPositions{convertedPositionsCache(timeStep, setBased)};
        getStateByTimeStep(timeStep)->setLonPosition(convertedPositions[timeStep][curRefLaneCCS][0]);
        getStateByTimeStep(timeStep)->setLatPosition(convertedPositions[timeStep][curRefLaneCCS][1]);
        getStateByTimeStep(timeStep)->setCurvilinearOrientation(convertedPositions[timeStep][curRefLaneCCS][2]);
    } catch (...) {
        throw std::runtime_error(ccsErrorMsg(timeStep, curRefLaneCCS, "convertPointToCurvilinear"));
    }
}

void Obstacle::interpolateAcceleration(size_t timeStep, double timeStepSize) const {
    if (getStateByTimeStep(timeStep)->getValidStates().acceleration)
        return;
    if (!timeStepExists(timeStep - 1)) {
        getStateByTimeStep(timeStep)->setAcceleration(0);
        return;
    }
    double curVelocity{getStateByTimeStep(timeStep)->getVelocity()};
    double prevVelocity{getStateByTimeStep(timeStep - 1)->getVelocity()};
    getStateByTimeStep(timeStep)->setAcceleration((curVelocity - prevVelocity) / timeStepSize);
}

void Obstacle::setOccupiedLanes(const std::vector<std::shared_ptr<Lane>> &lanes, size_t timeStep, bool setBased) {
    auto &occupiedLanes{getOccupiedLanesCache(timeStep, setBased)};
    if (occupiedLanes.count(timeStep) == 0)
        occupiedLanes[timeStep] = lanes;
}

void Obstacle::setObstacleRole(ObstacleRole type) { obstacleRole = type; }

void Obstacle::setOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep, bool setBased) {
    auto &occupiedLanes{getOccupiedLanesCache(timeStep, setBased)};
    auto lanelets{getOccupiedLaneletsRoadByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lane>> occLanes{lane_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters.getFieldOfViewRear(), sensorParameters.getFieldOfViewFront(),
        roadNetworkParameters.numIntersectionsPerDirectionLaneGeneration,
        {getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition()})};
    occupiedLanes[timeStep] = occLanes;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::getOccupiedLaneletsRoadByShape(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    std::set<size_t> relevantLanelets1;
    std::set<size_t> relevantLanelets2;
    const auto lets{getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &let : lets) {
        relevantLanelets1.insert(let->getId());
    }

    // special case where obstacle drives mostly on lanelets in other driving direction (see
    // SetReferenceGeneralScenario4 test case)
    size_t startTime{0};
    if (timeStep < roadNetworkParameters.relevantTimeIntervalSize)
        startTime = firstTimeStep;
    else
        startTime = std::max(firstTimeStep, timeStep - roadNetworkParameters.relevantTimeIntervalSize);

    // find all paths for all occupied initial lanelets to all occupied final lanelets
    for (const auto &initialLet : getOccupiedLaneletsByShape(roadNetwork, startTime))
        for (const auto &finalLet : getOccupiedLaneletsByShape(
                 roadNetwork, std::min(finalTimeStep, timeStep + roadNetworkParameters.relevantTimeIntervalSize))) {
            auto path{roadNetwork->getTopologicalMap()->findPaths(initialLet->getId(), finalLet->getId(), true)};
            relevantLanelets2.insert(path.begin(), path.end());
        }

    // get lanelet objects for relevant lanelets and also consider adjacent lanelets
    for (const auto &letBase : relevantLanelets2)
        for (const auto &adj : lanelet_operations::adjacentLanelets(roadNetwork->findLaneletById(letBase), true))
            relevantLanelets2.insert(adj->getId());

    // todo part above could be extracted and only computed on demand (usually when
    // trajectoryPrediction.trajectoryPrediction.changes) restrict lanelets to the ones which are currently occupied
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &letBase : getOccupiedLaneletsByShape(roadNetwork, timeStep))
        if (relevantLanelets1.find(letBase->getId()) != relevantLanelets1.end() and
            relevantLanelets2.find(letBase->getId()) != relevantLanelets2.end())
            lanelets.push_back(letBase);

    return lanelets;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedRoadLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                                  size_t timeStep) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep)};
    if (occLanes.size() == 1)
        return occLanes;

    auto ref{getReferenceLane(roadNetwork, timeStep)};
    auto occLanelets{getOccupiedLaneletsRoadByShape(roadNetwork, timeStep)};
    std::vector<std::shared_ptr<Lane>> relevantLanes;
    for (const auto &lane : occLanes) {
        if (lane->getId() == ref->getId() or lane->isPartOf(ref) or ref->isPartOf(lane)) {
            relevantLanes.push_back(lane);
            continue;
        }
        // check whether lanelet is adjacent to reference lane
        if (lanelet_operations::areLaneletsInDirectlyAdjacentLanes(getReferenceLane(roadNetwork, timeStep), lane,
                                                                   occLanelets))
            relevantLanes.push_back(lane);
    }
    return relevantLanes;
}

std::vector<std::shared_ptr<Lane>>
Obstacle::getOccupiedLanesDrivingDirection(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    auto occLanes{getOccupiedLanes(roadNetwork, timeStep)};
    if (occLanes.size() == 1)
        return occLanes;

    std::vector<std::shared_ptr<Lane>> relevantLanes;
    const auto occLanelets{getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep)};
    for (const auto &lanelet : occLanes) {
        if (lanelet->getId() == getReferenceLane(roadNetwork, timeStep)->getId()) {
            relevantLanes.push_back(lanelet);
            continue;
        }
        // check whether lanelet is adjacent to reference lane
        if (lanelet_operations::areLaneletsInDirectlyAdjacentLanes(getReferenceLane(roadNetwork, timeStep), lanelet,
                                                                   occLanelets))
            relevantLanes.push_back(lanelet);
    }
    return relevantLanes;
}

time_step_map_t<std::vector<std::shared_ptr<Lane>>> &Obstacle::getOccupiedLanesCache(const size_t timeStep,
                                                                                     const bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLanes;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLanes;
    return trajectoryPrediction.obstacleCache.occupiedLanes;
}

std::vector<std::shared_ptr<Lane>> Obstacle::getOccupiedLanes(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                              size_t timeStep, const bool setBased) {
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        timeStep = getCurrentState()->getTimeStep(); // only valid state for set-based prediction
    auto &occupiedLanes{getOccupiedLanesCache(timeStep, setBased)};
    if (occupiedLanes[timeStep].empty())
        setOccupiedLanes(roadNetwork, timeStep);
    return occupiedLanes[timeStep];
}

std::vector<std::shared_ptr<Lane>>
Obstacle::getOccupiedLanesAndAdjacent(const std::shared_ptr<RoadNetwork> &roadNetwork, size_t timeStep) {
    std::set<size_t> relevantLanelets;
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    auto lets{getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep)};
    if (lets.empty())
        lets = getOccupiedLaneletsByShape(roadNetwork, timeStep);
    for (const auto &let : lets) {
        relevantLanelets.insert(let->getId());
        for (const auto &let2 : lanelet_operations::adjacentLanelets(let, false))
            relevantLanelets.insert(let2->getId());
    }
    for (const auto &id : relevantLanelets)
        lanelets.push_back(roadNetwork->findLaneletById(id));

    auto occLanes{lane_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters.getFieldOfViewRear(), sensorParameters.getFieldOfViewFront(),
        roadNetworkParameters.numIntersectionsPerDirectionLaneGeneration,
        {getStateByTimeStep(timeStep)->getXPosition(), getStateByTimeStep(timeStep)->getYPosition()})};

    return occLanes;
}

void Obstacle::computeLanes(const std::shared_ptr<RoadNetwork> &roadNetwork, bool considerHistory) {
    const size_t timeStamp{recordedStates.currentState->getTimeStep()};
    auto lanelets{getOccupiedLaneletsByShape(roadNetwork, timeStamp)};
    auto lanes{lane_operations::createLanesBySingleLanelets(
        lanelets, roadNetwork, sensorParameters.getFieldOfViewRear(), sensorParameters.getFieldOfViewFront(),
        roadNetworkParameters.numIntersectionsPerDirectionLaneGeneration, {})};
    setOccupiedLanes(lanes, timeStamp);
    if (!isStatic()) {
        for (const auto &time : getPredictionTimeSteps())
            setOccupiedLanes(roadNetwork, time);
        if (considerHistory)
            for (const auto &time : getHistoryTimeSteps())
                setOccupiedLanes(roadNetwork, time);
    }
}

void Obstacle::setCurvilinearStates(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (!recordedStates.currentState->getValidStates().lonPosition)
        convertPointToCurvilinear(roadNetwork, recordedStates.currentState->getTimeStep());
    if (!isStatic())
        for (const auto &timeStep : getPredictionTimeSteps())
            if (!getStateByTimeStep(timeStep)->getValidStates().lonPosition)
                convertPointToCurvilinear(roadNetwork, timeStep);
}

const polygon_type Obstacle::getFov() { return sensorParameters.getFieldOfViewPolygon(); }

time_step_map_t<std::vector<std::shared_ptr<Lanelet>>> &
Obstacle::getOccupiedLaneletsDrivingDirCache(size_t timeStep, bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLaneletsDrivingDir;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLaneletsDrivingDir;
    return trajectoryPrediction.obstacleCache.occupiedLaneletsDrivingDir;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                     time_step_t timeStep, bool setBased) {
    auto &occupiedLaneletsDrivingDir{getOccupiedLaneletsDrivingDirCache(timeStep, setBased)};
    if (occupiedLaneletsDrivingDir.find(timeStep) != occupiedLaneletsDrivingDir.end())
        return occupiedLaneletsDrivingDir[timeStep];

    if (setBased and !setBasedPrediction.setBasedPrediction.empty() and timeStep > getCurrentState()->getTimeStep()) {
        // use only lanelets which are part of the lane of the current time step
        std::set<size_t> ids;
        std::vector<std::shared_ptr<Lanelet>> lanelets;
        auto lanes{getOccupiedLanes(roadNetwork, getCurrentState()->getTimeStep(), setBased)};
        for (const auto &la : setOccupiedLaneletsByShape(roadNetwork, timeStep, setBased)) {
            for (const auto &lane : lanes) {
                if (lane->containsLanelet({la->getId()}) and ids.find(la->getId()) == ids.end()) {
                    lanelets.push_back(la);
                    ids.insert(la->getId());
                }
            }
        }
        occupiedLaneletsDrivingDir[timeStep] = lanelets;
        return occupiedLaneletsDrivingDir[timeStep];
    }

    std::set<size_t> relevantLanelets1;
    std::set<size_t> relevantLanelets2;
    auto occLanelets{getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &la : occLanelets)
        if (std::abs(geometric_operations::subtractOrientations(
                la->getOrientationAtPosition(getStateByTimeStep(timeStep)->getXPosition(),
                                             getStateByTimeStep(timeStep)->getYPosition()),
                getStateByTimeStep(timeStep)->getGlobalOrientation())) < 0.785)
            relevantLanelets1.insert(la->getId());
        else
            relevantLanelets1.insert(la->getId());

    // special case where obstacle drives mostly on lanelets in other driving direction (see
    // SetReferenceGeneralScenario4 test case)
    size_t startTime{0};
    if (timeStep < roadNetworkParameters.relevantTimeIntervalSize)
        startTime = firstTimeStep;
    else
        startTime = std::max(firstTimeStep, timeStep - roadNetworkParameters.relevantTimeIntervalSize);

    // find all paths for all occupied initial lanelets to all occupied final lanelets
    for (const auto &initialLet : getOccupiedLaneletsByShape(roadNetwork, startTime))
        for (const auto &finalLet : getOccupiedLaneletsByShape(
                 roadNetwork, std::min(finalTimeStep, timeStep + roadNetworkParameters.relevantTimeIntervalSize))) {
            auto path{roadNetwork->getTopologicalMap()->findPaths(initialLet->getId(), finalLet->getId(), true)};
            relevantLanelets2.insert(path.begin(), path.end());
        }

    // todo part above could be extracted and only computed on demand (usually when
    // trajectoryPrediction.trajectoryPrediction.changes) restrict lanelets to the ones which are currently occupied
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &letBase : getOccupiedLaneletsByShape(roadNetwork, timeStep))
        if (relevantLanelets1.find(letBase->getId()) != relevantLanelets1.end() and
            relevantLanelets2.find(letBase->getId()) != relevantLanelets2.end())
            lanelets.push_back(letBase);

    occupiedLaneletsDrivingDir[timeStep] = lanelets;
    return occupiedLaneletsDrivingDir[timeStep];
}

time_step_map_t<std::vector<std::shared_ptr<Lanelet>>> &
Obstacle::getOccupiedLaneletsNotDrivingDirCache(size_t timeStep, bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.occupiedLaneletsNotDrivingDir;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.occupiedLaneletsNotDrivingDir;
    return trajectoryPrediction.obstacleCache.occupiedLaneletsNotDrivingDir;
}

std::vector<std::shared_ptr<Lanelet>>
Obstacle::setOccupiedLaneletsNotDrivingDirectionByShape(const std::shared_ptr<RoadNetwork> &roadNetwork,
                                                        time_step_t timeStep, bool setBased) {
    auto &occupiedLaneletsNotDrivingDir{getOccupiedLaneletsNotDrivingDirCache(timeStep, setBased)};
    if (occupiedLaneletsNotDrivingDir.find(timeStep) != occupiedLaneletsNotDrivingDir.end())
        return occupiedLaneletsNotDrivingDir[timeStep];

    auto occ = getOccupiedLaneletsDrivingDirectionByShape(roadNetwork, timeStep);
    auto all = getOccupiedLaneletsByShape(roadNetwork, timeStep);

    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lanelet : all) {
        if (std::none_of(occ.begin(), occ.end(),
                         [lanelet](std::shared_ptr<Lanelet> &occL) { return lanelet->getId() == occL->getId(); }))
            lanelets.emplace_back(lanelet);
    }

    occupiedLaneletsNotDrivingDir[timeStep] = lanelets;
    return occupiedLaneletsNotDrivingDir[timeStep];
}

void Obstacle::setCurrentSignalState(const std::shared_ptr<SignalState> &state) {
    recordedStates.currentSignalState = state;
}

void Obstacle::appendSignalStateToSeries(const std::shared_ptr<SignalState> &state) {
    trajectoryPrediction.signalSeries.insert(
        std::pair<size_t, std::shared_ptr<SignalState>>(state->getTimeStep(), state));
}

void Obstacle::appendSignalStateToHistory(const std::shared_ptr<SignalState> &state) {
    recordedStates.signalSeriesHistory.insert(
        std::pair<size_t, std::shared_ptr<SignalState>>(state->getTimeStep(), state));
}
const signal_state_map_t &Obstacle::getSignalSeries() const { return trajectoryPrediction.signalSeries; }

const signal_state_map_t &Obstacle::getSignalSeriesHistory() const { return recordedStates.signalSeriesHistory; }

const std::shared_ptr<SignalState> &Obstacle::getCurrentSignalState() const {
    return recordedStates.currentSignalState;
}

bool Obstacle::isStatic() const { return obstacleRole == ObstacleRole::STATIC; }

double Obstacle::drivenTrajectoryDistance() const {
    std::vector<vertex> polyline{
        {recordedStates.currentState->getXPosition(), recordedStates.currentState->getYPosition()}};
    for (const auto &state : trajectoryAsVector())
        polyline.push_back({state->getXPosition(), state->getYPosition()});
    return geometric_operations::computePathLengthFromPolyline(polyline).back();
}
std::vector<std::shared_ptr<State>> Obstacle::trajectoryAsVector() const {
    std::vector<std::shared_ptr<State>> trajectory;
    for (size_t timeStep{getFirstTrajectoryTimeStep()}; timeStep <= finalTimeStep; ++timeStep)
        if (trajectoryPrediction.trajectoryPrediction.find(timeStep) != trajectoryPrediction.trajectoryPrediction.end())
            trajectory.push_back(trajectoryPrediction.trajectoryPrediction.at(timeStep));
    return trajectory;
}

size_t Obstacle::getFirstTimeStep() const { return firstTimeStep; }

size_t Obstacle::getFinalTimeStep() const { return finalTimeStep; }

std::vector<double> Obstacle::getFrontXYCoordinates(time_step_t timeStep, bool setBased) {
    auto &frontXYPositions{getFrontXYCoordinatesCache(timeStep, setBased)};
    if (frontXYPositions.find(timeStep) != frontXYPositions.end())
        return frontXYPositions[timeStep];

    std::shared_ptr<State> state = getStateByTimeStep(timeStep);
    double frontX = getGeoShape().getLength() / 2 * cos(state->getGlobalOrientation()) + state->getXPosition();
    double frontY = getGeoShape().getLength() / 2 * sin(state->getGlobalOrientation()) + state->getYPosition();
    std::vector<double> result{frontX, frontY};
    frontXYPositions[timeStep] = result;
    return frontXYPositions[timeStep];
}

time_step_map_t<std::vector<double>> &Obstacle::getFrontXYCoordinatesCache(time_step_t timeStep, bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.frontXYPositions;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.frontXYPositions;
    return trajectoryPrediction.obstacleCache.frontXYPositions;
}

time_step_map_t<std::vector<double>> &Obstacle::getBackXYCoordinatesCache(time_step_t timeStep, bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.backXYPositions;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.backXYPositions;
    return trajectoryPrediction.obstacleCache.backXYPositions;
}

std::vector<double> Obstacle::getBackXYCoordinates(time_step_t timeStep, bool setBased) {
    auto &backXYPositions{getBackXYCoordinatesCache(timeStep, setBased)};
    if (backXYPositions.find(timeStep) != backXYPositions.end())
        return backXYPositions[timeStep];

    std::shared_ptr<State> state = getStateByTimeStep(timeStep);
    double backX = getGeoShape().getLength() / 2 * cos(state->getGlobalOrientation() + M_PI) + state->getXPosition();
    double backY = getGeoShape().getLength() / 2 * sin(state->getGlobalOrientation() + M_PI) + state->getYPosition();
    std::vector<double> result{backX, backY};
    backXYPositions[timeStep] = result;
    return backXYPositions[timeStep];
}

void Obstacle::setFirstLastTimeStep() {
    if (this->trajectoryPrediction.trajectoryPrediction.empty())
        finalTimeStep = this->recordedStates.currentState->getTimeStep();
    else
        finalTimeStep = std::max_element(this->trajectoryPrediction.trajectoryPrediction.begin(),
                                         this->trajectoryPrediction.trajectoryPrediction.end())
                            ->first;
    if (recordedStates.trajectoryHistory.empty())
        firstTimeStep = this->recordedStates.currentState->getTimeStep();
    else
        firstTimeStep =
            std::min_element(recordedStates.trajectoryHistory.begin(), recordedStates.trajectoryHistory.end())->first;
}

bool Obstacle::historyPassed(size_t currentTimeStep) const {
    if (currentTimeStep - firstTimeStep > timeParameters.getRelevantHistorySize())
        return true;
    return false;
}

void Obstacle::propagate() {
    size_t newCur{getCurrentState()->getTimeStep() + 1};
    if (timeStepExists(newCur)) {
        updateCurrentState(getStateByTimeStep(newCur));
        trajectoryPrediction.trajectoryPrediction.erase(trajectoryPrediction.trajectoryPrediction.find(newCur));
    }
}

occupancy_map_t Obstacle::getSetBasedPrediction() const { return setBasedPrediction.setBasedPrediction; }

double Obstacle::getVelocity(size_t timeStep, bool setBased, bool min) const {
    if (setBased and timeStep > recordedStates.currentState->getTimeStep() and
        !setBasedPrediction.setBasedPrediction.empty()) {
        if (min) {
            // addition since aMinLong already negative
            return std::max(recordedStates.currentState->getVelocity() +
                                (timeStep - recordedStates.currentState->getTimeStep()) * 0.1 *
                                    actuatorParameters.getAminLong(),
                            0.0);
        }
        return std::min(recordedStates.currentState->getVelocity() +
                            (timeStep - recordedStates.currentState->getTimeStep()) * timeParameters.getTimeStepSize() *
                                actuatorParameters.getAmaxLong(),
                        actuatorParameters.getVmax());
    }
    return getStateByTimeStep(timeStep)->getVelocity();
}

double Obstacle::getAcceleration(size_t timeStep, bool setBased, bool min) const {
    if (setBased and !setBasedPrediction.setBasedPrediction.empty() and timeStep > getCurrentState()->getTimeStep()) {
        if (min) {
            if (getVelocity(timeStep, setBased, false) == 0)
                return 0;
            return actuatorParameters.getAminLong();
        }
        if (getVelocity(timeStep, setBased, min) == actuatorParameters.getVmax())
            return 0;
        return actuatorParameters.getAmaxLong();
    }
    return getStateByTimeStep(timeStep)->getAcceleration();
}

time_step_map_t<std::map<size_t, double>> &Obstacle::getLateralDistanceToObjectCache(size_t timeStep,
                                                                                     bool setBased) const {
    if (timeStep <= recordedStates.currentState->getTimeStep())
        return recordedStates.occupancyRecorded.lateralDistanceToObjects;
    if (setBased and !setBasedPrediction.setBasedPrediction.empty())
        return setBasedPrediction.obstacleCache.lateralDistanceToObjects;
    return trajectoryPrediction.obstacleCache.lateralDistanceToObjects;
}

double Obstacle::getLateralDistanceToObstacle(time_step_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                              const std::shared_ptr<RoadNetwork> &roadnetwork, bool setBased) {
    auto &lateralDistanceToObjects{getLateralDistanceToObjectCache(timeStep, setBased)};
    if (lateralDistanceToObjects.find(timeStep) != lateralDistanceToObjects.end() &&
        lateralDistanceToObjects[timeStep].find(obs->getId()) != lateralDistanceToObjects[timeStep].end())
        return lateralDistanceToObjects[timeStep][obs->getId()];

    const double leftThis = leftD(roadnetwork, timeStep);
    const double rightThis = rightD(roadnetwork, timeStep);

    const double leftOther =
        obs->leftD(timeStep, getReferenceLane(roadnetwork, timeStep)->getCurvilinearCoordinateSystem());
    const double rightOther =
        obs->rightD(timeStep, getReferenceLane(roadnetwork, timeStep)->getCurvilinearCoordinateSystem());

    const double min = std::min(abs(rightThis - leftOther), abs(leftThis - rightOther));

    lateralDistanceToObjects[timeStep][obs->getId()] = min;

    return min;
}

void Obstacle::clearCache() {
    recordedStates.clearCache();
    trajectoryPrediction.clearCache();
    setBasedPrediction.clearCache();
}

void Obstacle::setReferenceLane(const std::shared_ptr<Lane> &refLane) {
    if (dynamicRef) {
        dynamicRef = false;
        recordedStates.occupancyRecorded.referenceLane.clear();
        setBasedPrediction.obstacleCache.referenceLane.clear();
        trajectoryPrediction.obstacleCache.referenceLane.clear();
    }
    recordedStates.occupancyRecorded.referenceLane[0] = refLane;
}
