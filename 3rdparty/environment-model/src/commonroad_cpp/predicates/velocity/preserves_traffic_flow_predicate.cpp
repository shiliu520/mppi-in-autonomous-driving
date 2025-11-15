#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/velocity/preserves_traffic_flow_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool PreservesTrafficFlowPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    double vMax{std::min({regulatory_elements_utils::speedLimitSuggested(
                              obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep),
                              TrafficSignTypes::MAX_SPEED, parameters.getParam("desiredInterstateVelocity"),
                              parameters.getParam("desiredUrbanVelocity")),
                          regulatory_elements_utils::typeSpeedLimit(obstacleK->getObstacleType()),
                          parameters.getParam("brakingSpeedLimit"), parameters.getParam("fovSpeedLimit"),
                          parameters.getParam("roadConditionSpeedLimit")})};
    return (vMax - obstacleK->getStateByTimeStep(timeStep)->getVelocity()) < parameters.getParam("minVelocityDif");
}

double PreservesTrafficFlowPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("PreservesTrafficFlowPredicate does not support robust evaluation!");
}

Constraint PreservesTrafficFlowPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("PreservesTrafficFlowPredicate does not support constraint evaluation!");
}
PreservesTrafficFlowPredicate::PreservesTrafficFlowPredicate() : CommonRoadPredicate(false) {}
