#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <commonroad_cpp/auxiliaryDefs/interval.h>
#include <commonroad_cpp/goal_state.h>
#include <commonroad_cpp/obstacle/initial_state.h>

class RoadNetwork;
class Obstacle;

class PlanningProblem {
  public:
    /**
     * Constructor for planning problem.
     *
     * @param planningProblemID ID of the planning problem.
     * @param initialState Initial state.
     * @param goalStates Goal states.
     * @param egoId Optional ego vehicle ID.
     */
    PlanningProblem(uint32_t planningProblemId, std::shared_ptr<InitialState> initialState,
                    std::vector<GoalState> goalStates, std::optional<uint32_t> egoId = std::nullopt);

    PlanningProblem() = default;

    /**
     * Getter for planning problem ID.
     *
     * @return Planning Problem ID.
     */
    [[nodiscard]] uint32_t getId() const;

    /**
     * Getter for initial state.
     *
     * @return Initial State.
     */
    [[nodiscard]] std::shared_ptr<InitialState> getInitialState() const;

    /**
     * Getter for goal states.
     *
     * @return Goal States.
     */
    [[nodiscard]] const std::vector<GoalState> &getGoalStates() const;

    /**
     * Getter for ego vehicle ID.
     *
     * @return Ego Vehicle ID.
     */
    [[nodiscard]] std::optional<uint32_t> getEgoId() const;

  private:
    uint32_t planningProblemId;                 //**< Planning Problem ID. */
    std::shared_ptr<InitialState> initialState; //**< Initial state. */
    std::vector<GoalState> goalStates;          //**< Goal states. */
    std::optional<uint32_t> egoId;              //**< Ego ID. */

    // TODO: Add scenario tags
};
