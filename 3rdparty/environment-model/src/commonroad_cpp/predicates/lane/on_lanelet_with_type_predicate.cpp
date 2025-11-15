
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/predicates/lane/on_lanelet_with_type_predicate.h>

bool OnLaneletWithTypePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, setBased);
    return std::any_of(lanelets.begin(), lanelets.end(),
                       [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet->hasLaneletType(
                               lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(0)));
                       });
}

double OnLaneletWithTypePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    throw std::runtime_error("OnLaneletWithTypePredicate does not support robust evaluation!");
}

Constraint OnLaneletWithTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnLaneletWithTypePredicate does not support constraint evaluation!");
}
OnLaneletWithTypePredicate::OnLaneletWithTypePredicate() : CommonRoadPredicate(false) {}
