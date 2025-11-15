#pragma once

#include <memory>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/auxiliaryDefs/timer.h"
#include "commonroad_cpp/predicates/predicate_parameter_collection.h"

class Lane;
class World;

/**
 * Interface for a lane-based predicate.
 */
class CommonRoadLanePredicate {
  public:
    virtual ~CommonRoadLanePredicate() = default;
    /**
     * Default constructor for lane predicate class without parameter.
     */
    CommonRoadLanePredicate() = default;

    /**
     * Copy constructor.
     *
     * @param predicate Predicate to copy.
     */
    CommonRoadLanePredicate(const CommonRoadLanePredicate &predicate) = delete;

    /**
     * Virtual function for the boolean evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param laneK Pointer to the kth lane.
     * @param laneP Pointer to the pth lane. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @return Boolean indicating satisfaction of the predicate.
     */
    virtual bool booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                   const std::shared_ptr<Lane> &laneK, const std::shared_ptr<Lane> &laneP,
                                   const std::vector<std::string> &additionalFunctionParameters = {"0.0"}) = 0;

    /**
     * Function for the statistical evaluation of a predicate.
     *
     * @param timeStep Time step of interest.
     * @param world Contains road network, ego vehicle, and obstacle list.
     * @param laneK Pointer to the kth lane.
     * @param evaluationTimer Timer which measures computation time.
     * @param statistics Statistic of predicates.
     * @param laneP Pointer to the pth lane. This is an optional parameter.
     * @param additionalFunctionParameters Additional parameters.
     * @return Boolean indicating satisfaction
     */
    bool statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                    const std::shared_ptr<Lane> &laneK, const std::shared_ptr<Timer> &evaluationTimer,
                                    const std::shared_ptr<PredicateStatistics> &statistics,
                                    const std::shared_ptr<Lane> &laneP = {},
                                    const std::vector<std::string> &additionalFunctionParameters = {"0.0"});

    /**
     * Getter for parameters.
     *
     * @return Struct containing all parameters.
     */
    [[nodiscard]] PredicateParameters &getParameters();

  protected:
    PredicateParameters parameters; //**< Struct containing parameters of all predicates. */
};

extern std::map<std::string, std::shared_ptr<CommonRoadLanePredicate>>
    lanePredicates; //**< List of all lane predicates **/
