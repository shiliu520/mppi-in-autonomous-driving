#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/intersection/on_incoming_left_of_predicate.h>
#include <commonroad_cpp/roadNetwork/intersection/incoming_group.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

bool OnIncomingLeftOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    // check whether obstacles are on incoming lanelet
    std::vector<std::shared_ptr<Lanelet>> laneletsK =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    if (!std::any_of(laneletsK.begin(), laneletsK.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
            return lanelet->hasLaneletType(LaneletType::incoming);
        }))
        return false;
    std::vector<std::shared_ptr<Lanelet>> laneletsP =
        obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    if (!std::any_of(laneletsP.begin(), laneletsP.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
            return lanelet->hasLaneletType(LaneletType::incoming);
        }))
        return false;

    auto laneK = obstacleK->getReferenceLane(world->getRoadNetwork(), timeStep);
    auto laneP = obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep);
    for (const auto &letK : laneK->getContainedLanelets()) {
        if (!letK->hasLaneletType(LaneletType::incoming))
            continue;
        auto incomingK{world->getRoadNetwork()->findIncomingGroupByLanelet(letK)};
        if (incomingK->getIsLeftOf() == nullptr)
            return false; // e.g, T-intersection or intersection with single incoming group
        for (const auto &letP : laneP->getContainedLanelets()) {
            if (!letP->hasLaneletType(LaneletType::incoming))
                continue;
            auto incomingP{world->getRoadNetwork()->findIncomingGroupByLanelet(letP)};
            if (incomingK->getIsLeftOf()->getId() == incomingP->getId())
                return true;
        }
    }
    return false;
}

double OnIncomingLeftOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    throw std::runtime_error("OnIncomingLeftOfPredicate does not support robust evaluation!");
}

Constraint OnIncomingLeftOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const std::vector<std::string> &additionalFunctionParameters,
                                                           bool setBased) {
    throw std::runtime_error("OnIncomingLeftOfPredicate does not support constraint evaluation!");
}
OnIncomingLeftOfPredicate::OnIncomingLeftOfPredicate() : CommonRoadPredicate(true) {}
