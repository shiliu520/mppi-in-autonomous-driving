#include "commonroad_cpp/roadNetwork/intersection/crossing_group.h"
#include "commonroad_cpp/roadNetwork/intersection/incoming_group.h"
#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <algorithm>
#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection_operations.h>
#include <utility>

Intersection::Intersection(const size_t intersectionId, std::vector<std::shared_ptr<IncomingGroup>> incomingGroups,
                           std::vector<std::shared_ptr<OutgoingGroup>> outgoingGroups,
                           std::vector<std::shared_ptr<CrossingGroup>> crossingGroups)
    : id(intersectionId), incomings(std::move(incomingGroups)), outgoings(std::move(outgoingGroups)),
      crossings(std::move(crossingGroups)) {
    determineIntersectionType();
}

size_t Intersection::getId() const { return id; }

void Intersection::setId(const size_t num) { id = num; }

const std::vector<std::shared_ptr<IncomingGroup>> &Intersection::getIncomingGroups() const { return incomings; }

void Intersection::addIncomingGroup(const std::shared_ptr<IncomingGroup> &incoming) {
    incomings.push_back(incoming);
    memberLanelets.clear();
    intersectionTypes.clear();
}

void Intersection::addOutgoingGroup(const std::shared_ptr<OutgoingGroup> &outgoing) {
    outgoings.push_back(outgoing);
    memberLanelets.clear();
    intersectionTypes.clear();
}

const std::vector<std::shared_ptr<OutgoingGroup>> &Intersection::getOutgoingGroups() const { return outgoings; }

void Intersection::setIncomingGroups(const std::vector<std::shared_ptr<IncomingGroup>> &incs) { incomings = incs; }

void Intersection::setOutgoingGroups(const std::vector<std::shared_ptr<OutgoingGroup>> &outs) { outgoings = outs; }

const std::vector<std::shared_ptr<Lanelet>> &
Intersection::getMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    if (memberLanelets.empty())
        computeMemberLanelets(roadNetwork);
    return memberLanelets;
}

void Intersection::computeMemberLanelets(const std::shared_ptr<RoadNetwork> &roadNetwork) {
    memberLanelets = {};
    // collect outgoings
    for (const auto &incom : incomings) {
        for (const auto &letInc : incom->getIncomingLanelets()) {
            letInc->addLaneletType(LaneletType::incoming);
            memberLanelets.push_back(letInc);
            std::vector<std::shared_ptr<Lanelet>> allOutgoings;
            allOutgoings.insert(allOutgoings.end(), incom->getLeftOutgoings().begin(), incom->getLeftOutgoings().end());
            allOutgoings.insert(allOutgoings.end(), incom->getStraightOutgoings().begin(),
                                incom->getStraightOutgoings().end());
            allOutgoings.insert(allOutgoings.end(), incom->getRightOutgoings().begin(),
                                incom->getRightOutgoings().end());

            for (const auto &letOut : incom->getLeftOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionLeftOutgoing);
                letOut->addLaneletType(LaneletType::intersection);
                letOut->addLaneletType(LaneletType::left);
                memberLanelets.push_back(letOut);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (std::find(allOutgoings.begin(), allOutgoings.end(), let) != allOutgoings.end())
                        break;
                    if (!std::any_of(
                            memberLanelets.begin(), memberLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == let->getId(); })) {
                        memberLanelets.push_back(let);
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::left);
                    }
                }
            }
            for (const auto &letOut : incom->getStraightOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionStraightOutgoing);
                letOut->addLaneletType(LaneletType::straight);
                letOut->addLaneletType(LaneletType::intersection);
                memberLanelets.push_back(letOut);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (std::find(allOutgoings.begin(), allOutgoings.end(), let) != allOutgoings.end())
                        break;
                    if (!std::any_of(
                            memberLanelets.begin(), memberLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == let->getId(); })) {
                        memberLanelets.push_back(let);
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::straight);
                    }
                }
            }
            for (const auto &letOut : incom->getRightOutgoings()) {
                letOut->addLaneletType(LaneletType::intersectionRightOutgoing);
                letOut->addLaneletType(LaneletType::right);
                letOut->addLaneletType(LaneletType::intersection);
                memberLanelets.push_back(letOut);
                auto path{roadNetwork->getTopologicalMap()->findPaths(letInc->getId(), letOut->getId(), false)};
                for (const auto &pathLet : path) {
                    auto let{roadNetwork->findLaneletById(pathLet)};
                    if (std::find(allOutgoings.begin(), allOutgoings.end(), let) != allOutgoings.end())
                        break;
                    if (!std::any_of(
                            memberLanelets.begin(), memberLanelets.end(),
                            [let](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == let->getId(); })) {
                        memberLanelets.push_back(let);
                        let->addLaneletType(LaneletType::intersection);
                        let->addLaneletType(LaneletType::right);
                    }
                }
            }
        }
        intersection_operations::findLeftOf(incom, roadNetwork);
    }
}
void Intersection::setCrossingGroups(const std::vector<std::shared_ptr<CrossingGroup>> &cros) { crossings = cros; }

void Intersection::addCrossingGroup(const std::shared_ptr<CrossingGroup> &crossing) { crossings.push_back(crossing); }

const std::vector<std::shared_ptr<CrossingGroup>> &Intersection::getCrossingGroups() const { return crossings; }

bool Intersection::hasIntersectionType(const IntersectionType type) {
    if (intersectionTypes.empty())
        determineIntersectionType();
    if (intersectionTypes.find(type) != intersectionTypes.end())
        return true;
    return false;
}

void Intersection::determineIntersectionType() {
    bool hasFourWayStop{false};
    if (incomings.size() == 4) {
        intersectionTypes.insert(IntersectionType::FOUR_WAY_INTERSECTION);
        // if lanelet from each incoming references a STOP sign, it is 4 way stop
        if (std::all_of(incomings.begin(), incomings.end(), [](const std::shared_ptr<IncomingGroup> &incoming) {
                return std::any_of(incoming->getIncomingLanelets().begin(), incoming->getIncomingLanelets().end(),
                                   [](const std::shared_ptr<Lanelet> &la) {
                                       return la->hasTrafficSign(TrafficSignTypes::STOP_4_WAY);
                                   });
            })) {
            intersectionTypes.insert(IntersectionType::FOUR_WAY_STOP_INTERSECTION);
            hasFourWayStop = true;
        }
    } else if (incomings.size() == 3) {
        bool hasTIntersection{false};
        const auto incoming_1 = incomings.at(0)->getIncomingLanelets().at(0);
        const auto incoming_2 = incomings.at(1)->getIncomingLanelets().at(0);
        const auto incoming_3 = incomings.at(2)->getIncomingLanelets().at(0);

        // get the orientation from each incoming lanelet with the vertices
        const auto orientation_incoming_1 = geometric_operations::getOrientationInDeg(incoming_1);
        const auto orientation_incoming_2 = geometric_operations::getOrientationInDeg(incoming_2);
        const auto orientation_incoming_3 = geometric_operations::getOrientationInDeg(incoming_3);

        // now get through the three cases how the incomings can be located
        // first case: T is incoming_1
        if (geometric_operations::is90Deg(orientation_incoming_1, orientation_incoming_2) &&
            geometric_operations::is90Deg(orientation_incoming_1, orientation_incoming_3)) {
            if (geometric_operations::is180Deg(orientation_incoming_2, orientation_incoming_3)) {
                intersectionTypes.insert(IntersectionType::T_INTERSECTION);
                hasTIntersection = true;
            }
        }
        // second case: T is incoming_2
        if (!hasTIntersection and geometric_operations::is90Deg(orientation_incoming_2, orientation_incoming_1) &&
            geometric_operations::is90Deg(orientation_incoming_2, orientation_incoming_3)) {
            if (geometric_operations::is180Deg(orientation_incoming_1, orientation_incoming_3)) {
                intersectionTypes.insert(IntersectionType::T_INTERSECTION);
                hasTIntersection = true;
            }
        }
        // third case: T is incoming_3
        if (!hasTIntersection and geometric_operations::is90Deg(orientation_incoming_3, orientation_incoming_1) &&
            geometric_operations::is90Deg(orientation_incoming_3, orientation_incoming_2)) {
            if (geometric_operations::is180Deg(orientation_incoming_1, orientation_incoming_2)) {
                intersectionTypes.insert(IntersectionType::T_INTERSECTION);
            }
        }
    }
    if (!hasFourWayStop) {
        if (std::all_of(incomings.begin(), incomings.end(), [](const std::shared_ptr<IncomingGroup> &incoming) {
                return std::all_of(incoming->getIncomingLanelets().begin(), incoming->getIncomingLanelets().end(),
                                   [](const std::shared_ptr<Lanelet> &la) {
                                       return la->getTrafficLights().empty() and la->getTrafficSigns().empty() and
                                              la->getStopLine() == nullptr;
                                   });
            })) {
            intersectionTypes.insert(IntersectionType::UNCONTROLLED_INTERSECTION);
        }
    }
    if (intersectionTypes.empty())
        intersectionTypes.insert(IntersectionType::UNKNOWN);
}
