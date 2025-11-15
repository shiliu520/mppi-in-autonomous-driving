#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/velocity/slow_other_vehicle_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool SlowOtherVehiclePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    double vMax{std::min(
        {regulatory_elements_utils::speedLimitSuggested(
             obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased),
             TrafficSignTypes::MAX_SPEED, parameters.getParam("desiredInterstateVelocity"),
             parameters.getParam("desiredUrbanVelocity")),
         regulatory_elements_utils::typeSpeedLimit(obstacleK->getObstacleType()),
         parameters.getParam("roadConditionSpeedLimit")})};
    return vMax - obstacleP->getVelocity(timeStep, setBased, false) >= parameters.getParam("minVelocityDif");
}

double SlowOtherVehiclePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    throw std::runtime_error("SlowLeadingVehiclePredicate does not support robust evaluation!");
}

Constraint SlowOtherVehiclePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const std::vector<std::string> &additionalFunctionParameters,
                                                           bool setBased) {
    throw std::runtime_error("SlowLeadingVehiclePredicate does not support constraint evaluation!");
}

SlowOtherVehiclePredicate::SlowOtherVehiclePredicate() : CommonRoadPredicate(false) {}
