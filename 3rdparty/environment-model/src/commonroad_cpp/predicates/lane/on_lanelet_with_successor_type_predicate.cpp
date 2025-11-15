#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/lane/on_lanelet_with_successor_type_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

OnLaneletWithSuccessorTypePredicate::OnLaneletWithSuccessorTypePredicate() : CommonRoadPredicate(false) {}

bool OnLaneletWithSuccessorTypePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    std::vector<std::shared_ptr<Lanelet>> occupiedLanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);

    for (auto &lanelet : occupiedLanelets) {
        if (lanelet->hasLaneletType(lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(0))))
            return true;
        auto idx{lanelet->findClosestIndex(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                           obstacleK->getStateByTimeStep(timeStep)->getYPosition(), true)};
        std::vector<std::vector<std::shared_ptr<Lanelet>>> successors =
            lane_operations::combineLaneletAndSuccessorsToLane(
                lanelet, obstacleK->getSensorParameters().getFieldOfViewFront(),
                obstacleK->getRoadNetworkParameters().numIntersectionsPerDirectionLaneGeneration, {},
                -lanelet->getPathLength().at(idx));
        for (auto &vec : successors) {
            for (auto &lane : vec) {
                if (lane->hasLaneletType(
                        lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(0))))
                    return true;
            }
        }
    }
    return false;
}

double OnLaneletWithSuccessorTypePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("On Lanelet With Successor Type Predicate does not support robust evaluation!");
}

Constraint OnLaneletWithSuccessorTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("On Lanelet With Successor Type Predicate does not support constraint evaluation!");
}
