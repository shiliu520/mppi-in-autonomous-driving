#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/braking_at_intersection_possible_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

bool BrakingAtIntersectionPossiblePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    double distanceIntersection{0.0};

    double brakingDistance{std::pow(obstacleK->getStateByTimeStep(timeStep)->getVelocity(), 2) /
                           (2 * abs(parameters.getParam("intersectionBrakingPossible")))};
    for (const auto &lane : lane_operations::createLanesBySingleLanelets(
             obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep), world->getRoadNetwork(),
             obstacleK->getSensorParameters().getFieldOfViewRear(),
             obstacleK->getSensorParameters().getFieldOfViewFront(),
             obstacleK->getRoadNetworkParameters().numIntersectionsPerDirectionLaneGeneration, {})) {
        for (const auto &letLane : lane->getContainedLanelets()) {
            if (letLane->hasLaneletType(LaneletType::incoming)) {
                auto curvPos{lane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
                    letLane->getCenterVertices().back().x, letLane->getCenterVertices().back().y)};
                auto ownPos{lane->getCurvilinearCoordinateSystem()->convertToCurvilinearCoords(
                    obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                    obstacleK->getStateByTimeStep(timeStep)->getYPosition())};
                distanceIntersection = curvPos.x() - ownPos.x();
                break;
            }
        }
        if (distanceIntersection != 0.0)
            break;
    }

    return brakingDistance < distanceIntersection;
}

Constraint BrakingAtIntersectionPossiblePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("BrakingWithAccelerationPossible does not support robust evaluation!");
}

double BrakingAtIntersectionPossiblePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("BrakingWithAccelerationPossible does not support robust evaluation!");
}

BrakingAtIntersectionPossiblePredicate::BrakingAtIntersectionPossiblePredicate() : CommonRoadPredicate(false) {}
