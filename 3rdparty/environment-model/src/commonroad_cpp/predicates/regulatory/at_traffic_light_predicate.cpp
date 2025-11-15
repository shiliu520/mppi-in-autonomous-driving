#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h"
#include <commonroad_cpp/predicates/regulatory/at_traffic_light_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>

bool AtTrafficLightPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters,
                                                bool setBased) {
    return regulatory_elements_utils::atTrafficLightDirState(
        timeStep, obstacleK, world->getRoadNetwork(),
        regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)),
        TrafficLight::matchTrafficLightState(additionalFunctionParameters.at(1)));
}
double AtTrafficLightPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    throw std::runtime_error("AtRedTrafficLightPredicate does not support robust evaluation!");
}
Constraint AtTrafficLightPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    throw std::runtime_error("AtRedTrafficLightPredicate does not support constraint evaluation!");
}
AtTrafficLightPredicate::AtTrafficLightPredicate() : CommonRoadPredicate(false) {}
