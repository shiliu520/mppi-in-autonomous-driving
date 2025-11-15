#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/velocity/keeps_lane_speed_limit_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool KeepsLaneSpeedLimitPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    double vMaxLane{regulatory_elements_utils::speedLimit(
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep),
        TrafficSignTypes::MAX_SPEED)};
    return vMaxLane >= obstacleK->getStateByTimeStep(timeStep)->getVelocity();
}

double KeepsLaneSpeedLimitPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    throw std::runtime_error("KeepsLaneSpeedLimitPredicate does not support robust evaluation!");
}

Constraint KeepsLaneSpeedLimitPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("KeepsLaneSpeedLimitPredicate does not support constraint evaluation!");
}
KeepsLaneSpeedLimitPredicate::KeepsLaneSpeedLimitPredicate() : CommonRoadPredicate(false) {}
