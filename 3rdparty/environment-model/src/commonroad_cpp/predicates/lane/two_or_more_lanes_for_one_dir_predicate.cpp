#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/lane/two_or_more_lanes_for_one_dir_predicate.h>

bool TwoOrMoreLanesForOneDirPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(lanelets.begin(), lanelets.end(), [](const auto &lanelet) {
        return (lanelet->getAdjacentLeft().adj && !lanelet->getAdjacentLeft().oppositeDir) ||
               (lanelet->getAdjacentRight().adj && !lanelet->getAdjacentRight().oppositeDir);
    });
}
double TwoOrMoreLanesForOneDirPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    throw std::runtime_error("TwoOrMoreLanesForOneDirPredicate does not support robust evaluation!");
}
Constraint TwoOrMoreLanesForOneDirPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("TwoOrMoreLanesForOneDirPredicate does not support constraint evaluation!");
}
TwoOrMoreLanesForOneDirPredicate::TwoOrMoreLanesForOneDirPredicate() : CommonRoadPredicate(false) {}
