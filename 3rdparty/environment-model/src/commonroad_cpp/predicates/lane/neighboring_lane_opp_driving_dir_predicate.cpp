#include "commonroad_cpp/predicates/lane/neighboring_lane_opp_driving_dir_predicate.h"
#include "commonroad_cpp/auxiliaryDefs/regulatory_elements.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

bool NeighboringLaneOppDrivingDirPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    auto occLanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};
    auto dir{regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0))};
    return std::any_of(occLanelets.begin(), occLanelets.end(),
                       [dir](const std::shared_ptr<Lanelet> &la) { return la->getAdjacent(dir).oppositeDir; });
}

double NeighboringLaneOppDrivingDirPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("NeighboringLaneOppDrivingDirPredicate does not support robust evaluation!");
}

Constraint NeighboringLaneOppDrivingDirPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("NeighboringLaneOppDrivingDirPredicate does not support constraint evaluation!");
}
NeighboringLaneOppDrivingDirPredicate::NeighboringLaneOppDrivingDirPredicate() : CommonRoadPredicate(false) {}
