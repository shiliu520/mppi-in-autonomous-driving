#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/in_intersection_conflict_area_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool InIntersectionConflictAreaPredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {

    auto simLaneletsK{
        obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep, setBased)};
    std::vector<std::shared_ptr<Lanelet>> laneletsP;
    if (setBased and
        !obstacleP->getSetBasedPrediction().empty()) // we do not check for initial state as we compute ref lane
        for (const auto &lane : obstacleP->getOccupiedLanes(world->getRoadNetwork(), timeStep, setBased))
            laneletsP.insert(laneletsP.end(), lane->getContainedLanelets().begin(), lane->getContainedLanelets().end());
    else {
        auto lane{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};
        laneletsP = lane->getContainedLanelets();
    }

    for (const auto &letP : laneletsP) {
        for (const auto &letK : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)) {
            if (!letK->hasLaneletType(LaneletType::intersection))
                continue;
            if (letK->getId() == letP->getId() and
                !std::any_of(simLaneletsK.begin(), simLaneletsK.end(),
                             [letK](const std::shared_ptr<Lanelet> &letSim) {
                                 return letSim->hasLaneletType(LaneletType::intersection) and
                                        letSim->getId() == letK->getId();
                             }) and
                !std::any_of(simLaneletsK.begin(), simLaneletsK.end(), [letP](const std::shared_ptr<Lanelet> &letSim) {
                    return letSim->hasLaneletType(LaneletType::intersection) and
                           intersection_operations::checkSameIncoming(
                               letP, letSim, SensorParameters::dynamicDefaults().getFieldOfViewFront(),
                               1); // extend only until next intersection
                }))
                return true;
        }
    }

    return false;
}

double InIntersectionConflictAreaPredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support robust evaluation!");
}

Constraint InIntersectionConflictAreaPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InIntersectionConflictAreaPredicate does not support constraint evaluation!");
}

InIntersectionConflictAreaPredicate::InIntersectionConflictAreaPredicate() : CommonRoadPredicate(true) {}
