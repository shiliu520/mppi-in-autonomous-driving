#include "commonroad_cpp/goal_state.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <memory>
#include <utility>
#include <vector>

#include <commonroad_cpp/planning_problem.h>

PlanningProblem::PlanningProblem(uint32_t planningProblemId, std::shared_ptr<InitialState> initialState,
                                 std::vector<GoalState> goalStates, std::optional<uint32_t> egoId)
    : planningProblemId{planningProblemId}, initialState{initialState}, goalStates{goalStates}, egoId{egoId} {}

uint32_t PlanningProblem::getId() const { return planningProblemId; }

std::shared_ptr<InitialState> PlanningProblem::getInitialState() const { return initialState; }

const std::vector<GoalState> &PlanningProblem::getGoalStates() const { return goalStates; }

std::optional<uint32_t> PlanningProblem::getEgoId() const { return egoId; }
