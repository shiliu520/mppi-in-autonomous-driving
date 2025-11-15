#include <algorithm>

#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

bool intersection_operations::onIncoming(size_t timeStep, const std::shared_ptr<Obstacle> &obs,
                                         const std::shared_ptr<RoadNetwork> &roadNetwork) {
    auto lanelets{obs->getOccupiedLaneletsByShape(roadNetwork, timeStep)};
    for (const auto &let : lanelets)
        if (std::any_of(let->getLaneletTypes().begin(), let->getLaneletTypes().end(),
                        [](const LaneletType typ) { return typ == LaneletType::incoming; }))
            return true;

    return false;
}

bool intersection_operations::checkSameIncoming(const std::shared_ptr<Lanelet> &letK,
                                                const std::shared_ptr<Lanelet> &letP, double fov,
                                                int numIntersections) {
    auto simLaneletsK{lane_operations::combineLaneLanelets(
        lane_operations::combineLaneletAndPredecessorsToLane(letK, fov, numIntersections))};
    auto simLaneletsP{lane_operations::combineLaneLanelets(
        lane_operations::combineLaneletAndPredecessorsToLane(letP, fov, numIntersections))};
    for (const auto &laK : simLaneletsK) {
        if (!laK->hasLaneletType(LaneletType::incoming))
            continue;
        for (const auto &adjLet : lanelet_operations::adjacentLanelets(laK)) {
            if (std::any_of(simLaneletsP.begin(), simLaneletsP.end(), [adjLet](const std::shared_ptr<Lanelet> &exLet) {
                    return exLet->getId() == adjLet->getId();
                }))
                return true;
        }
    }
    return false;
}

void intersection_operations::findLeftOf(const std::shared_ptr<IncomingGroup> &origin,
                                         const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (!origin->getRightOutgoings().empty()) {
        auto out = roadNetwork->findOutgoingGroupByLanelet(origin->getRightOutgoings()[0]);
        if (out)
            origin->setIsLeftOf(roadNetwork->findIncomingGroupByOutgoingGroup(out));
    }
}

std::shared_ptr<Intersection> intersection_operations::currentIntersection(size_t timeStep,
                                                                           const std::shared_ptr<World> &world,
                                                                           const std::shared_ptr<Obstacle> &obs) {
    auto lanelets{obs->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};

    for (const auto &intersection : world->getRoadNetwork()->getIntersections()) {
        for (const auto &lanelet : intersection->getMemberLanelets(world->getRoadNetwork())) {
            if (std::any_of(lanelets.begin(), lanelets.end(), [lanelet](const std::shared_ptr<Lanelet> &occLane) {
                    return occLane->getId() == lanelet->getId();
                }))
                return intersection;
        }
    }
    return nullptr;
}

std::shared_ptr<IncomingGroup> intersection_operations::currentIncoming(size_t timeStep,
                                                                        const std::shared_ptr<World> &world,
                                                                        const std::shared_ptr<Obstacle> &obs) {
    const auto all_intersection{world->getRoadNetwork()->getIntersections()};
    const auto lanelets{obs->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};

    for (const auto &intersection : all_intersection) {
        for (const auto &incoming : intersection->getIncomingGroups()) {
            for (const auto &lanelet : incoming->getIncomingLanelets()) {
                if (std::any_of(lanelets.begin(), lanelets.end(),
                                [lanelet](const std::shared_ptr<Lanelet> &occ_lanelet) {
                                    return occ_lanelet->getId() == lanelet->getId();
                                }))
                    return incoming;
            }
        }
    }

    return nullptr;
}

IntersectionType intersection_operations::matchStringToIntersectionType(const std::string &type) {
    std::string str{type};
    std::transform(str.begin(), str.end(), str.begin(), ::toupper);
    if (str == "T_INTERSECTION")
        return IntersectionType::T_INTERSECTION;
    else if (str == "FOUR_WAY_STOP_INTERSECTION")
        return IntersectionType::FOUR_WAY_STOP_INTERSECTION;
    else if (str == "FOUR_WAY_INTERSECTION")
        return IntersectionType::FOUR_WAY_INTERSECTION;
    else if (str == "UNCONTROLLED_INTERSECTION")
        return IntersectionType::UNCONTROLLED_INTERSECTION;
    return IntersectionType::UNKNOWN;
}
