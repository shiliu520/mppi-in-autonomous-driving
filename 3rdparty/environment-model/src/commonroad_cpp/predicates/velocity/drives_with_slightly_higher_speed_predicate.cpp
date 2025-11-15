#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/drives_with_slightly_higher_speed_predicate.h>
#include <commonroad_cpp/world.h>

bool DrivesWithSlightlyHigherSpeedPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    double diff = obstacleK->getVelocity(timeStep, setBased, true) - obstacleP->getVelocity(timeStep, setBased, false);
    return 0 < diff and diff < parameters.getParam("slightlyHigherSpeedDifference");
}

Constraint DrivesWithSlightlyHigherSpeedPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("Drives With Slightly Higher Speed Predicate does not support constraint evaluation!");
}

double DrivesWithSlightlyHigherSpeedPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("Drives With Slightly Higher Speed Predicate does not support robust evaluation!");
}
DrivesWithSlightlyHigherSpeedPredicate::DrivesWithSlightlyHigherSpeedPredicate() : CommonRoadPredicate(true) {}
