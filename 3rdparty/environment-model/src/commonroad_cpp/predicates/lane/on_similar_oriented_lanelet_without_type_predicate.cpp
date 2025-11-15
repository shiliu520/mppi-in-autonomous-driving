#include "commonroad_cpp/geometry/geometric_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_without_type_predicate.h>

bool OnSimilarOrientedLaneletWithoutTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {

    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased)};
    return std::all_of(lanelets.begin(), lanelets.end(),
                       [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
                           return !lanelet->hasLaneletType(
                               lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(0)));
                       });
}

double OnSimilarOrientedLaneletWithoutTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithoutTypePredicate does not support robust evaluation!");
}

Constraint OnSimilarOrientedLaneletWithoutTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OnSimilarOrientedLaneletWithoutTypePredicate does not support constraint evaluation!");
}
OnSimilarOrientedLaneletWithoutTypePredicate::OnSimilarOrientedLaneletWithoutTypePredicate()
    : CommonRoadPredicate(false) {}
