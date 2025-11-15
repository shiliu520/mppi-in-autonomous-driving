#include "commonroad_cpp/roadNetwork/intersection/outgoing_group.h"

OutgoingGroup::OutgoingGroup(size_t groupID, std::vector<std::shared_ptr<Lanelet>> &outgoingLanelets,
                             std::optional<size_t> incomingGroupID)
    : groupID(groupID), outgoingLanelets(outgoingLanelets), incomingGroupID(incomingGroupID) {}

void OutgoingGroup::setId(size_t outId) { groupID = outId; }

size_t OutgoingGroup::getId() const { return groupID; }

void OutgoingGroup::addOutgoingLanelet(std::shared_ptr<Lanelet> &lanelet) { outgoingLanelets.push_back(lanelet); }

void OutgoingGroup::setOutgoingLanelets(const std::vector<std::shared_ptr<Lanelet>> &outLanelets) {
    outgoingLanelets = outLanelets;
}

std::vector<std::shared_ptr<Lanelet>> OutgoingGroup::getOutgoingLanelets() { return outgoingLanelets; }

std::optional<size_t> OutgoingGroup::getIncomingGroupID() { return incomingGroupID; }

void OutgoingGroup::setIncomingGroupID(size_t inId) { incomingGroupID = inId; }
