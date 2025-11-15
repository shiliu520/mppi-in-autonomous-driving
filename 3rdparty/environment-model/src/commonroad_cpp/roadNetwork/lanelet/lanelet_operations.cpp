#include <algorithm>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <utility>

#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <range/v3/all.hpp>

LaneletType lanelet_operations::matchStringToLaneletType(const std::string &type) {
    std::string str{type};
    std::transform(str.begin(), str.end(), str.begin(), toupper);
    str.erase(std::remove_if(str.begin(), str.end(), [](const char elem) { return elem == '_'; }), str.end());
    if (LaneletTypeNames.count(str) == 1)
        return LaneletTypeNames.at(str);
    throw std::logic_error("lanelet_operations::matchStringToLaneletType: Invalid lanelet type '" + str + "'!");
}

LineMarking lanelet_operations::matchStringToLineMarking(const std::string &type) {
    if (type == "solid")
        return LineMarking::solid;
    if (type == "dashed")
        return LineMarking::dashed;
    if (type == "solid_solid")
        return LineMarking::solid_solid;
    if (type == "dashed_dashed")
        return LineMarking::dashed_dashed;
    if (type == "solid_dashed")
        return LineMarking::solid_dashed;
    if (type == "dashed_solid")
        return LineMarking::dashed_solid;
    if (type == "curb")
        return LineMarking::curb;
    if (type == "lowered_curb")
        return LineMarking::lowered_curb;
    if (type == "broad_solid")
        return LineMarking::broad_solid;
    if (type == "broad_dashed")
        return LineMarking::broad_dashed;
    if (type == "no_marking")
        return LineMarking::no_marking;
    return LineMarking::unknown;
}

std::vector<LineMarking> lanelet_operations::matchStringToLineMarkingOptions(const std::string &type) {
    std::string str{type};
    std::transform(str.begin(), str.end(), str.begin(), tolower);
    if (str == "solid")
        return {LineMarking::solid, LineMarking::solid_solid, LineMarking::solid_dashed, LineMarking::broad_solid};
    if (str == "dashed")
        return {LineMarking::dashed, LineMarking::dashed_dashed, LineMarking::dashed_solid, LineMarking::broad_dashed};
    if (str == "broad")
        return {LineMarking::broad_solid, LineMarking::broad_dashed};
    if (str == "solid_solid")
        return {LineMarking::solid_solid};
    if (str == "dashed_dashed")
        return {LineMarking::dashed_dashed};
    if (str == "solid_dashed")
        return {LineMarking::solid_dashed};
    if (str == "dashed_solid")
        return {LineMarking::dashed_solid};
    if (str == "curb")
        return {LineMarking::curb};
    if (str == "lowered_curb")
        return {LineMarking::lowered_curb};
    if (str == "broad_solid")
        return {LineMarking::broad_solid};
    if (str == "broad_dashed")
        return {LineMarking::broad_dashed};
    if (str == "no_marking")
        return {LineMarking::no_marking};
    return {LineMarking::unknown};
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsRightOfLanelet(std::shared_ptr<Lanelet> lanelet,
                                                                                 const bool sameDirection) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{std::move(lanelet)};

    while (curLanelet->getAdjacentRight().adj != nullptr and
           !std::any_of(adjacentLanelets.begin(), adjacentLanelets.end(),
                        [curLanelet](const std::shared_ptr<Lanelet> &let) {
                            return let->getId() == curLanelet->getAdjacentRight().adj->getId();
                        }) and
           (!sameDirection or !curLanelet->getAdjacentRight().oppositeDir)) {
        adjacentLanelets.push_back(curLanelet->getAdjacentRight().adj);
        curLanelet = curLanelet->getAdjacentRight().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::laneletsLeftOfLanelet(std::shared_ptr<Lanelet> lanelet,
                                                                                const bool sameDirection) {
    std::vector<std::shared_ptr<Lanelet>> adjacentLanelets;
    auto curLanelet{std::move(lanelet)};

    while (curLanelet->getAdjacentLeft().adj != nullptr and
           !std::any_of(adjacentLanelets.begin(), adjacentLanelets.end(),
                        [curLanelet](const std::shared_ptr<Lanelet> &let) {
                            return let->getId() == curLanelet->getAdjacentLeft().adj->getId();
                        }) and
           (!sameDirection or !curLanelet->getAdjacentLeft().oppositeDir)) {
        adjacentLanelets.push_back(curLanelet->getAdjacentLeft().adj);
        curLanelet = curLanelet->getAdjacentLeft().adj;
    }
    return adjacentLanelets;
}

std::vector<std::shared_ptr<Lanelet>>
lanelet_operations::removeDuplicateLanelets(const std::shared_ptr<Lanelet> &lanelet,
                                            const std::vector<std::shared_ptr<Lanelet>> &leftLanelets,
                                            const std::vector<std::shared_ptr<Lanelet>> &rightLanelets) {
    std::initializer_list<std::shared_ptr<Lanelet>> initial_lanelet{lanelet};

    // remove duplicates
    std::vector<std::shared_ptr<Lanelet>> adjLanelets =
        ranges::concat_view(initial_lanelet, leftLanelets, rightLanelets) | ranges::to<std::vector>;
    sort(adjLanelets.begin(), adjLanelets.end());
    adjLanelets.erase(unique(adjLanelets.begin(), adjLanelets.end()), adjLanelets.end());
    return adjLanelets;
}

std::vector<std::shared_ptr<Lanelet>> lanelet_operations::adjacentLanelets(const std::shared_ptr<Lanelet> &lanelet,
                                                                           const bool sameDirection) {
    const auto leftLanelets{laneletsLeftOfLanelet(lanelet, sameDirection)};
    const auto rightLanelets{laneletsRightOfLanelet(lanelet, sameDirection)};
    return removeDuplicateLanelets(lanelet, leftLanelets, rightLanelets);
}

bool lanelet_operations::areLaneletsAdjacent(const std::shared_ptr<Lanelet> &laneletOne,
                                             const std::shared_ptr<Lanelet> &laneletTwo) {
    return (laneletOne->getAdjacentRight().adj != nullptr and
            laneletOne->getAdjacentRight().adj->getId() == laneletTwo->getId()) or
           (laneletOne->getAdjacentLeft().adj != nullptr and
            laneletOne->getAdjacentLeft().adj->getId() == laneletTwo->getId());
}

bool lanelet_operations::areLaneletsInDirectlyAdjacentLanes(
    const std::shared_ptr<Lane> &laneOne, const std::shared_ptr<Lane> &laneTwo,
    const std::vector<std::shared_ptr<Lanelet>> &relevantLanelets) {
    for (const auto &la1 : relevantLanelets) {
        for (const auto &la2 : relevantLanelets) {
            if (la1->getId() == la2->getId())
                continue;
            if (areLaneletsAdjacent(la1, la2))
                if ((laneOne->getContainedLaneletIDs().find(la1->getId()) != laneOne->getContainedLaneletIDs().end() and
                     laneTwo->getContainedLaneletIDs().find(la2->getId()) != laneTwo->getContainedLaneletIDs().end()) or
                    (laneTwo->getContainedLaneletIDs().find(la1->getId()) != laneTwo->getContainedLaneletIDs().end() and
                     laneOne->getContainedLaneletIDs().find(la2->getId()) != laneOne->getContainedLaneletIDs().end()))
                    return true;
        }
    }
    return false;
}

double lanelet_operations::roadWidth(const std::shared_ptr<Lanelet> &lanelet, double xPosition, double yPosition) {
    std::vector<std::shared_ptr<Lanelet>> adj_lanelets = adjacentLanelets(lanelet, false);

    auto lanelet_widths = adj_lanelets | ranges::views::transform([xPosition, yPosition](auto &adjLanelet) -> double {
                              return adjLanelet->getWidth(xPosition, yPosition);
                          });

    return ranges::accumulate(lanelet_widths, 0.0);
}

std::vector<std::shared_ptr<TrafficLight>>
lanelet_operations::activeTlsByLanelet(size_t timeStep, const std::shared_ptr<Lanelet> &lanelet) {
    return lanelet->getTrafficLights() | ranges::views::filter([timeStep](const auto &light) {
               return light->isActive() or light->getElementAtTime(timeStep).color != TrafficLightState::inactive;
           }) |
           ranges::to<std::vector>;
}

bool lanelet_operations::bicycleLaneNextToRoad(const std::shared_ptr<Lanelet> &lanelet) {
    auto is_road = [](const std::shared_ptr<Lanelet> &la) {
        return !la->hasLaneletType(LaneletType::bicycleLane) and !la->hasLaneletType(LaneletType::sidewalk);
    };

    if (!lanelet->hasLaneletType(LaneletType::bicycleLane))
        return false;

    std::shared_ptr<Lanelet> left = lanelet->getAdjacentLeft().adj;
    while (left) {
        if (is_road(left))
            return true;
        left = left->getAdjacentLeft().adj;
    }
    std::shared_ptr<Lanelet> right = lanelet->getAdjacentRight().adj;
    while (right) {
        if (is_road(right))
            return true;
        right = right->getAdjacentRight().adj;
    }
    return false;
}

bool lanelet_operations::anyLaneletsContainLineMarkingType(const std::vector<std::shared_ptr<Lanelet>> &lanelets,
                                                           const std::vector<LineMarking> &lineMarkingTypes,
                                                           const std::string &direction) {
    return std::any_of(
        lanelets.begin(), lanelets.end(), [lineMarkingTypes, direction](const std::shared_ptr<Lanelet> &lanelet) {
            return std::find(lineMarkingTypes.begin(), lineMarkingTypes.end(),
                             lanelet->getLineMarking(regulatory_elements_utils::matchDirections(direction))) !=
                   lineMarkingTypes.end();
        });
}
