#include <commonroad_cpp/roadNetwork/intersection/crossing_group.h>

CrossingGroup::CrossingGroup(size_t crossingId, const std::vector<std::shared_ptr<Lanelet>> &crossingLanelets,
                             size_t incomingGroupId, size_t outgoingGroupId)
    : crossingGroupID(crossingId), crossingGroupLanelets(crossingLanelets), incomingGroupID(incomingGroupId),
      outgoingGroupID(outgoingGroupId) {}

size_t CrossingGroup::crossingId() const { return crossingGroupID; }

const std::vector<std::shared_ptr<Lanelet>> &CrossingGroup::getCrossingGroupLanelets() const {
    return crossingGroupLanelets;
}

void CrossingGroup::setCrossingGroupId(size_t crossingId) { crossingGroupID = crossingId; }

void CrossingGroup::setCrossingGroupLanelets(const std::vector<std::shared_ptr<Lanelet>> &crossingLanelets) {
    crossingGroupLanelets = crossingLanelets;
}

void CrossingGroup::addCrossingLanelet(const std::shared_ptr<Lanelet> &crossingLanelet) {
    crossingGroupLanelets.push_back(crossingLanelet);
}

void CrossingGroup::setOutgoingGroupID(size_t outId) { outgoingGroupID = outId; }

std::optional<size_t> CrossingGroup::getOutgoingGroupID() { return outgoingGroupID; }

std::optional<size_t> CrossingGroup::getIncomingGroupID() { return incomingGroupID; }

void CrossingGroup::setIncomingGroupID(size_t incId) { incomingGroupID = incId; }
