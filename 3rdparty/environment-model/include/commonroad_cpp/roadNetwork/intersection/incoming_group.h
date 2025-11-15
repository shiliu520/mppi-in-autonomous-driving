#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <vector>

class Lanelet;

/**
 * Class representing an incoming of an intersection.
 */
class IncomingGroup {
  public:
    /**
     * Default constructor.
     */
    IncomingGroup() = default;

    /**
     * Constructor of incoming.
     *
     * @param incomingId Incoming ID.
     * @param incomingLanelets Reference to lanelets belonging to incoming.
     * @param isLeftOf Reference incoming orientated left.
     * @param straightOutgoings Reference to straight outgoing lanelets.
     * @param leftOutgoings Reference to left outgoing lanelets.
     * @param rightOutgoings Reference to right outgoing lanelets.
     * @param oncomings Reference to oncoming lanelets.
     * @param outgoingGroupID ID of corresponding outgoing group
     */
    IncomingGroup(size_t incomingId, std::vector<std::shared_ptr<Lanelet>> incomingLanelets,
                  std::shared_ptr<IncomingGroup> isLeftOf, std::vector<std::shared_ptr<Lanelet>> straightOutgoings,
                  std::vector<std::shared_ptr<Lanelet>> leftOutgoings,
                  std::vector<std::shared_ptr<Lanelet>> rightOutgoings, std::vector<std::shared_ptr<Lanelet>> oncomings,
                  std::optional<size_t> outgoingGroupID = {});

    /**
     * Getter for incoming ID.
     *
     * @return Id of incoming.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for lanelets belonging to incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getIncomingLanelets() const;

    /**
     * Getter for incoming which is left of this incoming.
     *
     * @return Pointer to incoming.
     */
    [[nodiscard]] const std::shared_ptr<IncomingGroup> &getIsLeftOf() const;

    /**
     * Setter of incoming ID.
     *
     * @param incomingId Id of incoming.
     */
    void setId(size_t incomingId);

    /**
     * Setter for lanelets belonging to incoming.
     *
     * @param incomingLanelets List of pointers to lanelets.
     */
    void setIncomingLanelets(const std::vector<std::shared_ptr<Lanelet>> &incomingLanelets);

    /**
     * Setter for incoming which is left of this incoming.
     *
     * @param isLeftOf Pointer to incoming.
     */
    void setIsLeftOf(const std::shared_ptr<IncomingGroup> &isLeftOf);

    /**
     * Getter for straight outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getStraightOutgoings() const;

    /**
     * Setter for straight outgoing lanelets of incoming.
     *
     * @param straightOutgoings List of pointers to lanelets.
     */
    void setStraightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &straightOutgoings);

    /**
     * Getter for left outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getLeftOutgoings() const;

    /**
     * Setter for left outgoing lanelets of incoming.
     *
     * @param leftOutgoings List of pointers to lanelets.
     */
    void setLeftOutgoings(const std::vector<std::shared_ptr<Lanelet>> &leftOutgoings);

    /**
     * Getter for right outgoing lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getRightOutgoings() const;

    /**
     * Setter for right outgoing lanelets of incoming.
     *
     * @param rightOutgoings List of pointers to lanelets.
     */
    void setRightOutgoings(const std::vector<std::shared_ptr<Lanelet>> &rightOutgoings);

    /**
     * Getter for oncoming lanelets of incoming.
     *
     * @return List of pointers to lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getOncomings() const;

    /**
     * Setter for oncoming lanelets of incoming.
     *
     * @param oncomings List of pointers to lanelets.
     */
    void setOncomings(const std::vector<std::shared_ptr<Lanelet>> &oncomings);

    /**
     * Adds incoming lanelet to list of incoming lanelets.
     *
     * @param incomingLanelet Incoming lanelet
     */
    void addIncomingLanelet(const std::shared_ptr<Lanelet> &incomingLanelet);

    /**
     * Adds straight outgoing to list of straight outgoings.
     *
     * @param straightOutgoing Straight outgoing
     */
    void addStraightOutgoing(const std::shared_ptr<Lanelet> &straightOutgoing);

    /**
     * Adds left outgoing to list of left outgoings.
     *
     * @param leftOutgoing Left outgoing
     */
    void addLeftOutgoing(const std::shared_ptr<Lanelet> &leftOutgoing);

    /**
     * Adds right outgoing to list of right outgoings.
     *
     * @param rightOutgoing Right outgoing
     */
    void addRightOutgoing(const std::shared_ptr<Lanelet> &rightOutgoing);

    /**
     * Adds oncoming to list of oncomings.
     *
     * @param oncoming Oncoming
     */
    void addOncoming(const std::shared_ptr<Lanelet> &oncoming);

    /**
     * Setter for OutgoingGroup
     *
     * @param outId size_t
     */
    void setOutgoingGroupID(size_t outId);

    /**
     * Getter for OutgoingGroup
     *
     * @return size_t ID.
     */
    std::optional<size_t> getOutgoingGroupID();

    std::vector<std::shared_ptr<Lanelet>> getAllSuccessorLeft();
    std::vector<std::shared_ptr<Lanelet>> getAllSuccessorRight();
    std::vector<std::shared_ptr<Lanelet>> getAllSuccessorStraight();
    std::vector<std::shared_ptr<Lanelet>> getAllLeftTurningLanelets();
    std::vector<std::shared_ptr<Lanelet>> getAllRightTurningLanelets();
    std::vector<std::shared_ptr<Lanelet>> getAllStraightGoingLanelets();

  private:
    size_t groupID;
    std::vector<std::shared_ptr<Lanelet>>
        incomingLanelets;                    //**< set of pointers to lanelets belonging to incoming group */
    std::shared_ptr<IncomingGroup> isLeftOf; //**< pointer to incoming which is left */
    std::vector<std::shared_ptr<Lanelet>>
        straightOutgoings; //**< set of pointers to straight outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>>
        leftOutgoings; //**< set of pointers to left outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>>
        rightOutgoings; //**< set of pointers to right outgoing lanelets of this incoming */
    std::vector<std::shared_ptr<Lanelet>> oncomings; //**< set of pointers to oncoming lanelets of this incoming */
    std::optional<size_t> outgoingGroupID;           //**< ID of corresponding outgoing group*/

    std::vector<std::shared_ptr<Lanelet>> collectIncomingSuccessors(std::deque<std::shared_ptr<Lanelet>> &candidates,
                                                                    bool considerIncomings);
};
