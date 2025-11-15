#pragma once

#include <memory>
#include <optional>
#include <vector>

class Lanelet;

class OutgoingGroup {
  public:
    /**
     * Default constructor.
     */
    OutgoingGroup() = default;

    /**
     * Constructor
     *
     * @param groupID if of OutgoingGroup
     * @param outgoingLanelets vector of pointers to Lanelets of outgoingGroup
     * @param incomingGroupID ID of corresponding incoming group
     */
    OutgoingGroup(size_t groupID, std::vector<std::shared_ptr<Lanelet>> &outgoingLanelets,
                  std::optional<size_t> incomingGroupID = {});

    /**
     * Setter for the ID
     *
     * @param outId size_t
     */
    void setId(size_t outId);

    /**
     * Getter for the ID
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Add pointer to Lanelet to outgoingLanelets of OutgoingGroup
     *
     * @param lanelet pointer to Lanelet
     */
    void addOutgoingLanelet(std::shared_ptr<Lanelet> &lanelet);

    /**
     * Setter for lanelets belonging to outgoingGroup
     *
     * @param outLanelets pointer to vector of lanelets
     */
    void setOutgoingLanelets(const std::vector<std::shared_ptr<Lanelet>> &outLanelets);

    /**
     * Getter for outgoingLanelets of OutgoingGroup
     *
     * @return vector of pointer to Lanelets
     */
    std::vector<std::shared_ptr<Lanelet>> getOutgoingLanelets();

    /**
     * Getter for incoming group ID
     *
     * @return ID of incoming group
     */
    std::optional<size_t> getIncomingGroupID();

    /**
     * Setter for the incoming group ID
     *
     * @param inId ID of incoming group.
     */
    void setIncomingGroupID(size_t inId);

  private:
    size_t groupID{0};
    std::vector<std::shared_ptr<Lanelet>>
        outgoingLanelets;                  /**< set of pointers to lanelets belonging to outgoing group */
    std::optional<size_t> incomingGroupID; /**< corresponding incoming group ID */
};
