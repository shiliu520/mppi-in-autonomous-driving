#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/position/lateral_distance_to_obstacle_above_threshold_predicate.h>
#include <commonroad_cpp/world.h>

LateralDistanceToObstacleAboveThresholdPredicate::LateralDistanceToObstacleAboveThresholdPredicate()
    : CommonRoadPredicate(true) {}

bool LateralDistanceToObstacleAboveThresholdPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    return obstacleK->getLateralDistanceToObstacle(timeStep, obstacleP, world->getRoadNetwork()) >=
           std::stod(additionalFunctionParameters.at(0));
}

double LateralDistanceToObstacleAboveThresholdPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("LateralDistanceToObstacleAboveThresholdPredicate does not support robust evaluation!");
}

Constraint LateralDistanceToObstacleAboveThresholdPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error(
        "LateralDistanceToObstacleAboveThresholdPredicate does not support constraint evaluation!");
}
