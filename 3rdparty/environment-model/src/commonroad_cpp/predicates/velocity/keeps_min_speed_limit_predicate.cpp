#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/keeps_min_speed_limit_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/world.h>

bool KeepsMinSpeedLimitPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    double vReqLane{regulatory_elements_utils::requiredVelocity(
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep),
        TrafficSignTypes::MIN_SPEED)};
    return vReqLane <= obstacleK->getStateByTimeStep(timeStep)->getVelocity();
}

double KeepsMinSpeedLimitPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("KeepsMinSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsMinSpeedLimitPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("KeepsMinSpeedLimitPredicate does not support constraint evaluation!");
}
KeepsMinSpeedLimitPredicate::KeepsMinSpeedLimitPredicate() : CommonRoadPredicate(false) {}
