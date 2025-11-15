#include <vector>

#include <commonroad_cpp/goal_state.h>

GoalState::GoalState(IntegerInterval time, std::optional<FloatInterval> velocity,
                     std::optional<FloatInterval> orientation, std::vector<GoalPosition> goalPositions)
    : time{time}, velocity{velocity}, orientation{orientation}, goalPositions{goalPositions} {}

IntegerInterval GoalState::getTime() const { return time; }

std::optional<FloatInterval> GoalState::getVelocity() const { return velocity; }

std::optional<FloatInterval> GoalState::getOrientation() const { return orientation; }

std::vector<GoalPosition> GoalState::getGoalPositions() const { return goalPositions; }
