#pragma once
#include "commonroad_cpp/lanePredicates/commonroad_lane.h"

/**
 * Interface for a lane-based predicate.
 */
class IsSameLanePredicate : public CommonRoadLanePredicate {
  public:
    /**
     * Constructor for IsSameLanePredicate.
     */
    IsSameLanePredicate() = default;
    /**
     * Function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param laneK Pointer to the kth lane.
     * @param laneP Pointer to the pth lane. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @return Boolean indicating satisfaction of the predicate.
     */
    bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Lane> &laneK,
                           const std::shared_ptr<Lane> &laneP,
                           const std::vector<std::string> &additionalFunctionParameters = {"0.0"}) override;
};
