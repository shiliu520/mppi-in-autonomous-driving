#include <commonroad_cpp/roadNetwork/intersection/incoming_group.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>

#include <algorithm>
#include <deque>
#include <utility>

size_t IncomingGroup::getId() const { return groupID; }

void IncomingGroup::setId(size_t index) { groupID = index; }

const std::vector<std::shared_ptr<Lanelet>> &IncomingGroup::getIncomingLanelets() const { return incomingLanelets; }

void IncomingGroup::setIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>> &incLa) {
    // each incoming lanelet should be of type incoming
    for (const auto &let : incLa)
        let->addLaneletType(LaneletType::incoming);
    incomingLanelets = incLa;
}

const std::shared_ptr<IncomingGroup> &IncomingGroup::getIsLeftOf() const { return isLeftOf; }

void IncomingGroup::setIsLeftOf(const std::shared_ptr<IncomingGroup> &leftOf) { isLeftOf = leftOf; }

const std::vector<std::shared_ptr<Lanelet>> &IncomingGroup::getStraightOutgoings() const { return straightOutgoings; }

void IncomingGroup::setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &straight) {
    straightOutgoings = straight;
}

const std::vector<std::shared_ptr<Lanelet>> &IncomingGroup::getLeftOutgoings() const { return leftOutgoings; }

void IncomingGroup::setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &left) { leftOutgoings = left; }

const std::vector<std::shared_ptr<Lanelet>> &IncomingGroup::getRightOutgoings() const { return rightOutgoings; }

void IncomingGroup::setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &right) { rightOutgoings = right; }

const std::vector<std::shared_ptr<Lanelet>> &IncomingGroup::getOncomings() const { return oncomings; }

void IncomingGroup::setOncomings(const std::vector<std::shared_ptr<Lanelet>> &oncomingLanelets) {
    oncomings = oncomingLanelets;
}
IncomingGroup::IncomingGroup(size_t incomingId, std::vector<std::shared_ptr<Lanelet>> incomingLanelets,
                             std::shared_ptr<IncomingGroup> isLeftOf,
                             std::vector<std::shared_ptr<Lanelet>> straightOutgoings,
                             std::vector<std::shared_ptr<Lanelet>> leftOutgoings,
                             std::vector<std::shared_ptr<Lanelet>> rightOutgoings,
                             std::vector<std::shared_ptr<Lanelet>> oncomings, std::optional<size_t> outgoingGroupID)
    : groupID(incomingId), incomingLanelets(std::move(incomingLanelets)), isLeftOf(std::move(isLeftOf)),
      straightOutgoings(std::move(straightOutgoings)), leftOutgoings(std::move(leftOutgoings)),
      rightOutgoings(std::move(rightOutgoings)), oncomings(std::move(oncomings)), outgoingGroupID(outgoingGroupID) {}

std::vector<std::shared_ptr<Lanelet>> IncomingGroup::getAllSuccessorLeft() {

    std::deque<std::shared_ptr<Lanelet>> candidates{leftOutgoings.begin(), leftOutgoings.end()};
    return collectIncomingSuccessors(candidates, false);
}

std::vector<std::shared_ptr<Lanelet>> IncomingGroup::getAllSuccessorRight() {

    std::deque<std::shared_ptr<Lanelet>> candidates{rightOutgoings.begin(), rightOutgoings.end()};
    return collectIncomingSuccessors(candidates, false);
}

std::vector<std::shared_ptr<Lanelet>> IncomingGroup::getAllSuccessorStraight() {

    std::deque<std::shared_ptr<Lanelet>> candidates{straightOutgoings.begin(), straightOutgoings.end()};
    return collectIncomingSuccessors(candidates, false);
}

std::vector<std::shared_ptr<Lanelet>> IncomingGroup::getAllLeftTurningLanelets() {

    std::deque<std::shared_ptr<Lanelet>> candidates{leftOutgoings.begin(), leftOutgoings.end()};
    return collectIncomingSuccessors(candidates, true);
}

std::vector<std::shared_ptr<Lanelet>> IncomingGroup::getAllRightTurningLanelets() {

    std::deque<std::shared_ptr<Lanelet>> candidates{rightOutgoings.begin(), rightOutgoings.end()};
    return collectIncomingSuccessors(candidates, true);
}

std::vector<std::shared_ptr<Lanelet>> IncomingGroup::getAllStraightGoingLanelets() {

    std::deque<std::shared_ptr<Lanelet>> candidates{straightOutgoings.begin(), straightOutgoings.end()};
    return collectIncomingSuccessors(candidates, true);
}

std::vector<std::shared_ptr<Lanelet>>
IncomingGroup::collectIncomingSuccessors(std::deque<std::shared_ptr<Lanelet>> &candidates, bool considerIncomings) {
    std::vector<std::shared_ptr<Lanelet>> memberLanelets;
    std::vector<std::shared_ptr<Lanelet>> incomings;
    while (!candidates.empty()) {
        std::shared_ptr<Lanelet> pre{candidates.front()};
        candidates.pop_front();
        if (!std::any_of(memberLanelets.begin(), memberLanelets.end(),
                         [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); }) and
            (!std::any_of(incomingLanelets.begin(), incomingLanelets.end(),
                          [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); }))) {
            memberLanelets.push_back(pre);
            auto pres{memberLanelets.back()->getPredecessors()};
            candidates.insert(candidates.end(), pres.begin(), pres.end());
        } else if (considerIncomings and
                   std::any_of(incomingLanelets.begin(), incomingLanelets.end(),
                               [pre](const std::shared_ptr<Lanelet> &tmp) { return tmp->getId() == pre->getId(); }))
            incomings.push_back(pre);
        else
            continue;
    }
    if (considerIncomings)
        memberLanelets.insert(memberLanelets.end(), incomings.begin(), incomings.end());
    return memberLanelets;
}

void IncomingGroup::addIncomingLanelet(const std::shared_ptr<Lanelet> &incomingLanelet) {
    incomingLanelets.push_back(incomingLanelet);
}

void IncomingGroup::addStraightOutgoing(const std::shared_ptr<Lanelet> &straightOutgoing) {
    straightOutgoings.push_back(straightOutgoing);
}

void IncomingGroup::addLeftOutgoing(const std::shared_ptr<Lanelet> &leftOutgoing) {
    leftOutgoings.push_back(leftOutgoing);
}

void IncomingGroup::addRightOutgoing(const std::shared_ptr<Lanelet> &rightOutgoing) {
    rightOutgoings.push_back(rightOutgoing);
}

void IncomingGroup::addOncoming(const std::shared_ptr<Lanelet> &oncoming) { oncomings.push_back(oncoming); }

void IncomingGroup::setOutgoingGroupID(size_t outId) { outgoingGroupID = outId; }

std::optional<size_t> IncomingGroup::getOutgoingGroupID() { return outgoingGroupID; }
