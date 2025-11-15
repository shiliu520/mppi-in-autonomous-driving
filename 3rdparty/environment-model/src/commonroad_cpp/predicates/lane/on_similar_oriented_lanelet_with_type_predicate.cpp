#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_with_type_predicate.h>

bool OnSimilarOrientedLaneletWithTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {

    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased)};
    return std::any_of(lanelets.begin(), lanelets.end(),
                       [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                           return lanelet->hasLaneletType(
                               lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(0)));
                       });
}

double OnSimilarOrientedLaneletWithTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithTypePredicate does not support robust evaluation!");
}

Constraint OnSimilarOrientedLaneletWithTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithTypePredicate does not support constraint evaluation!");
}
OnSimilarOrientedLaneletWithTypePredicate::OnSimilarOrientedLaneletWithTypePredicate() : CommonRoadPredicate(false) {}
