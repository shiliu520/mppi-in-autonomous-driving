#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane_operations.h>

#include <range/v3/all.hpp>

multi_polygon_type computeCircle(double x, double y) {
    multi_polygon_type polygonShape{polygon_type{}};

    const std::vector boundingVertices{geometric_operations::addObjectDimensionsCircle({x, y}, 0.1)};
    polygonShape.at(0).outer().resize(boundingVertices.size() + 1);

    for (size_t i{0}; i < boundingVertices.size(); i++)
        polygonShape.at(0).outer()[i] = point_type{boundingVertices[i].x, boundingVertices[i].y};

    if (!boundingVertices.empty())
        polygonShape.at(0).outer().back() = point_type{boundingVertices[0].x, boundingVertices[0].y};

    return polygonShape;
}

std::shared_ptr<Lane> lane_operations::computeLaneFromTwoPoints(const vertex &start, const vertex &end,
                                                                const std::shared_ptr<RoadNetwork> &roadNetwork) {
    std::vector<std::shared_ptr<Lanelet>> initialLanelets;
    std::vector<std::shared_ptr<Lanelet>> finalLanelets;
    for (const auto &let : roadNetwork->findOccupiedLaneletsByShape(computeCircle(start.x, start.y))) {
        initialLanelets.push_back(let);
        auto adj{lanelet_operations::adjacentLanelets(let)};
        initialLanelets.insert(initialLanelets.end(), adj.begin(), adj.end());
    }
    for (const auto &let : roadNetwork->findOccupiedLaneletsByShape(computeCircle(end.x, end.y))) {
        finalLanelets.push_back(let);
        auto adj{lanelet_operations::adjacentLanelets(let)};
        finalLanelets.insert(finalLanelets.end(), adj.begin(), adj.end());
    }

    for (const auto &slet : initialLanelets) {
        for (const auto &elet : finalLanelets) {
            if (auto path{roadNetwork->getTopologicalMap()->findPaths(slet->getId(), elet->getId(), false)};
                !path.empty()) {
                std::vector<std::shared_ptr<Lanelet>> laneLanelets;
                for (const auto &let : path)
                    laneLanelets.push_back(roadNetwork->findLaneletById(let));
                return createLaneByContainedLanelets(laneLanelets, ++*roadNetwork->getIdCounterRef());
            }
        }
    }
    spdlog::error("lane_operations::computeLaneFromLine: No lane found between given start (" +
                  std::to_string(start.x) + "," + std::to_string(start.y) + ") and end (" + std::to_string(end.x) +
                  ", " + std::to_string(end.y) + ")");
    return nullptr;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>> lane_operations::combineLaneletAndSuccessorsToLane(
    const std::shared_ptr<Lanelet> &curLanelet, const double fov, int numIntersections,
    const std::vector<std::shared_ptr<Lanelet>> &containedLanelets, const double offset) {
    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector<std::shared_ptr<Lanelet>> laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{offset}; // neglect initial lanelet
    for (const auto &lanelet : laneletList)
        laneLength += lanelet->getPathLength().back();
    if (curLanelet->hasLaneletType(LaneletType::incoming))
        --numIntersections;

    // check whether it is the last lanelet of the lane, the lane contains no loop, and max length is reached
    if (!curLanelet->getSuccessors().empty() and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &lanelet) {
                         return curLanelet->getId() == lanelet->getId();
                     }) and
        laneLength < fov and numIntersections >= 0) {
        for (const auto &lanelet : curLanelet->getSuccessors()) {
            // neglect border since this is no real lane
            if (lanelet->hasLaneletType(LaneletType::border))
                continue;
            auto newLanes{combineLaneletAndSuccessorsToLane(lanelet, fov, numIntersections, laneletList, offset)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};
    return lanes;
}

std::vector<std::shared_ptr<Lane>> removeSubPartLanes(const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::vector<std::shared_ptr<Lane>> filteredLanes;

    for (const auto &lane : lanes) {
        bool isSubPart = false;
        for (const auto &otherLane : lanes) {
            if (lane != otherLane && otherLane->isPartOf(lane)) {
                isSubPart = true;
                break;
            }
        }
        if (!isSubPart) {
            filteredLanes.push_back(lane);
        }
    }
    return filteredLanes;
}

std::vector<std::vector<std::shared_ptr<Lanelet>>> lane_operations::combineLaneletAndPredecessorsToLane(
    const std::shared_ptr<Lanelet> &curLanelet, const double fov, int numIntersections,
    std::vector<std::shared_ptr<Lanelet>> containedLanelets, const double offset) {

    std::vector<std::vector<std::shared_ptr<Lanelet>>> lanes;
    std::vector laneletList{containedLanelets};
    laneletList.push_back(curLanelet);
    double laneLength{offset}; // neglect initial lanelet
    for (const auto &lanelet : laneletList)
        laneLength += lanelet->getPathLength().back();
    if (curLanelet->hasLaneletType(LaneletType::incoming))
        --numIntersections;

    // check whether it is the last lanelet of the lane, the lane contains no loop, and max length is reached
    if (!curLanelet->getPredecessors().empty() and
        !std::any_of(containedLanelets.begin(), containedLanelets.end(),
                     [curLanelet](const std::shared_ptr<Lanelet> &lanelet) {
                         return curLanelet->getId() == lanelet->getId();
                     }) and
        laneLength < fov and numIntersections >= 0) {
        for (const auto &lanelet : curLanelet->getPredecessors()) {
            // neglect border since this is no real lane
            if (lanelet->hasLaneletType(LaneletType::border))
                continue;
            auto newLanes{combineLaneletAndPredecessorsToLane(lanelet, fov, numIntersections, laneletList, offset)};
            lanes.insert(lanes.end(), newLanes.begin(), newLanes.end());
        }
    } else
        return {laneletList};

    return lanes;
}

std::vector<std::shared_ptr<Lane>>
lane_operations::createLanesBySingleLanelets(const std::vector<std::shared_ptr<Lanelet>> &initialLanelets,
                                             const std::shared_ptr<RoadNetwork> &roadNetwork, const double fovRear,
                                             const double fovFront, const int numIntersections, const vertex position) {
    std::vector<std::shared_ptr<Lane>> lanes;

    // create lanes
    for (const auto &lanelet : initialLanelets) {
        // neglect border since this is no real lane
        if (lanelet->hasLaneletType(LaneletType::border))
            continue;

        // check for existing lanes
        auto newLanes{roadNetwork->findLanesByBaseLanelet(lanelet->getId())};
        if (!newLanes.empty()) {
            bool existing{false};
            for (const auto &newLane : newLanes) {
                const auto idx{newLane->findClosestIndex(position.x, position.y, true)};
                const double rearLength{newLane->getPathLength().at(idx)};
                if (const double frontLength{newLane->getPathLength().back() - newLane->getPathLength().at(idx)};
                    !std::any_of(lanes.begin(), lanes.end(),
                                 [newLane](const std::shared_ptr<Lane> &lane) {
                                     return newLane->getContainedLaneletIDs() == lane->getContainedLaneletIDs();
                                 }) and
                    (frontLength > fovFront or newLane->getContainedLanelets().back()->getSuccessors().empty()) and
                    (rearLength > fovRear or newLane->getContainedLanelets().at(0)->getPredecessors().empty())) {
                    lanes.push_back(newLane);
                    existing = true;
                }
            }
            if (existing) // lane was already created based on this initial lanelet -> continue with next lanelet
                continue;
        }

        const auto idx{lanelet->findClosestIndex(position.x, position.y, true)};
        auto newLaneSuccessorParts{combineLaneletAndSuccessorsToLane(lanelet, fovFront, numIntersections, {},
                                                                     -lanelet->getPathLength().at(idx))};
        auto newLanePredecessorParts{
            combineLaneletAndPredecessorsToLane(lanelet, fovRear, numIntersections, {},
                                                lanelet->getPathLength().at(idx) - lanelet->getPathLength().back())};
        if (!newLaneSuccessorParts.empty() and !newLanePredecessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts)
                for (const auto &lanePre : newLanePredecessorParts) {
                    std::vector<std::shared_ptr<Lanelet>> containedLanelets{lanePre};
                    std::reverse(containedLanelets.begin(), containedLanelets.end());
                    containedLanelets.insert(containedLanelets.end(), laneSuc.begin() + 1, laneSuc.end());
                    if (auto newLane{
                            createLaneByContainedLanelets(containedLanelets, ++*roadNetwork->getIdCounterRef())};
                        newLane->getContainedLaneletIDs().find(lanelet->getId()) !=
                        newLane->getContainedLaneletIDs().end())
                        newLanes.push_back(newLane);
                }
        else if (!newLaneSuccessorParts.empty())
            for (const auto &laneSuc : newLaneSuccessorParts) {
                auto newLane{createLaneByContainedLanelets(laneSuc, ++*roadNetwork->getIdCounterRef())};
                if (newLane->getContainedLaneletIDs().find(lanelet->getId()) != newLane->getContainedLaneletIDs().end())
                    newLanes.push_back(newLane);
                newLanes.push_back(newLane);
            }
        else
            for (const auto &lanePre : newLanePredecessorParts) {
                auto newLane{createLaneByContainedLanelets(lanePre, ++*roadNetwork->getIdCounterRef())};
                if (newLane->getContainedLaneletIDs().find(lanelet->getId()) != newLane->getContainedLaneletIDs().end())
                    newLanes.push_back(newLane);
                newLanes.push_back(newLane);
            }
        if (newLanes.empty()) {
            // required if e.g., initial lanelet not part of new lane, e.g., in sidewalks where successors are wrongly
            // assigned
            auto newLanelet{Lanelet(++*roadNetwork->getIdCounterRef(), lanelet->getLeftBorderVertices(),
                                    lanelet->getRightBorderVertices(), {}, {}, lanelet->getLaneletTypes(),
                                    lanelet->getUsersOneWay(), lanelet->getUsersBidirectional())};
            std::vector clanelets{lanelet};
            newLanes.push_back(std::make_shared<Lane>(clanelets, newLanelet));
        }
        newLanes = roadNetwork->addLanes(newLanes, lanelet->getId());
        for (const auto &newLane : newLanes)
            if (!std::any_of(lanes.begin(), lanes.end(), [newLane](const std::shared_ptr<Lane> &lane) {
                    return newLane->getContainedLaneletIDs() == lane->getContainedLaneletIDs();
                }))
                lanes.push_back(newLane);
    }
    return removeSubPartLanes(lanes);
}

std::shared_ptr<Lane>
lane_operations::createLaneByContainedLanelets(const std::vector<std::shared_ptr<Lanelet>> &containedLanelets,
                                               const size_t newId) {
    std::set<ObstacleType> userOneWay;
    std::set<ObstacleType> userBidirectional;
    std::vector<vertex> centerVertices;
    std::vector<vertex> leftVertices;
    std::vector<vertex> rightVertices;
    std::set<LaneletType> typeList;
    long shift{0};

    std::vector<std::shared_ptr<Lanelet>> containedLaneletsWithoutDuplicates;

    for (const auto &lanelet : containedLanelets) {
        if (std::any_of(containedLaneletsWithoutDuplicates.begin(), containedLaneletsWithoutDuplicates.end(),
                        [lanelet](const std::shared_ptr<Lanelet> &let) { return let->getId() == lanelet->getId(); }))
            continue;
        bool reverse{false};
        if (!containedLaneletsWithoutDuplicates.empty() and
            geometric_operations::euclideanDistance2Dim(
                lanelet->getCenterVertices().back(),
                containedLaneletsWithoutDuplicates.back()->getCenterVertices().back()) < 0.1)
            reverse = true;
        if (!containedLaneletsWithoutDuplicates.empty() and
            geometric_operations::euclideanDistance2Dim(lanelet->getCenterVertices().front(), centerVertices.back()) >
                10)
            break;
        containedLaneletsWithoutDuplicates.push_back(lanelet);
        std::set_intersection(userOneWay.begin(), userOneWay.end(), lanelet->getUsersOneWay().begin(),
                              lanelet->getUsersOneWay().end(), std::inserter(userOneWay, userOneWay.begin()));

        std::set_intersection(userBidirectional.begin(), userBidirectional.end(),
                              lanelet->getUsersBidirectional().begin(), lanelet->getUsersBidirectional().end(),
                              std::inserter(userBidirectional, userBidirectional.begin()));

        if (reverse) {
            centerVertices.insert(centerVertices.end(), lanelet->getCenterVertices().rbegin() + shift,
                                  lanelet->getCenterVertices().rend());
            leftVertices.insert(leftVertices.end(), lanelet->getLeftBorderVertices().rbegin() + shift,
                                lanelet->getLeftBorderVertices().rend());
            rightVertices.insert(rightVertices.end(), lanelet->getRightBorderVertices().rbegin() + shift,
                                 lanelet->getRightBorderVertices().rend());
        } else {
            centerVertices.insert(centerVertices.end(), lanelet->getCenterVertices().begin() + shift,
                                  lanelet->getCenterVertices().end());

            leftVertices.insert(leftVertices.end(), lanelet->getLeftBorderVertices().begin() + shift,
                                lanelet->getLeftBorderVertices().end());

            rightVertices.insert(rightVertices.end(), lanelet->getRightBorderVertices().begin() + shift,
                                 lanelet->getRightBorderVertices().end());
        }

        std::set_intersection(typeList.begin(), typeList.end(), lanelet->getLaneletTypes().begin(),
                              lanelet->getLaneletTypes().end(), std::inserter(typeList, typeList.begin()));
        shift = 1;
    }

    auto newLanelet{Lanelet(newId, leftVertices, rightVertices, {}, {}, typeList, userOneWay, userBidirectional)};

    return std::make_shared<Lane>(containedLaneletsWithoutDuplicates, newLanelet);
}

std::vector<std::shared_ptr<Lanelet>>
lane_operations::extractLaneletsFromLanes(const std::vector<std::shared_ptr<Lane>> &lanes) {
    std::unordered_set<size_t> ids;
    auto lanelets = lanes |
                    ranges::views::transform([](const auto &lane) { return lane->getContainedLanelets(); })
                    // Join vector of vectors into vector
                    | ranges::actions::join;

    return lanelets
           // Only keep lanelets we haven't seen yet
           | ranges::views::filter([&ids](const auto &lanelet) {
                 const auto lid = lanelet->getId();
                 bool not_exists = ids.count(lid) == 0;
                 if (not_exists)
                     ids.insert(lid);
                 return not_exists;
             }) |
           ranges::to<std::vector>;
}

std::vector<std::shared_ptr<Lanelet>>
lane_operations::combineLaneLanelets(const std::vector<std::vector<std::shared_ptr<Lanelet>>> &lanes) {
    std::vector<std::shared_ptr<Lanelet>> lanelets;
    for (const auto &lane : lanes)
        for (const auto &let : lane)
            if (!std::any_of(lanelets.begin(), lanelets.end(),
                             [let](const std::shared_ptr<Lanelet> &exLet) { return exLet->getId() == let->getId(); }))
                lanelets.push_back(let);
    return lanelets;
}
