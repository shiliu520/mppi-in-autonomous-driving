#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include <commonroad_cpp/auxiliaryDefs/interval.h>
#include <commonroad_cpp/geometry/shape.h>
#include <commonroad_cpp/obstacle/state.h>

class RoadNetwork;
class Obstacle;

using GoalPosition = std::variant<uint32_t, std::shared_ptr<Shape>>;

class GoalState {
  public:
    /**
     * Constructor for goal state.
     *
     * @param time Goal time interval.
     * @param velocity Optional goal velocity interval.
     * @param orientation Optional goal orientation interval.
     * @param goalPositions Possible goal positions.
     */
    GoalState(IntegerInterval time, std::optional<FloatInterval> velocity, std::optional<FloatInterval> orientation,
              std::vector<GoalPosition> goalPositions);

    /**
     * Getter for goal time interval.
     *
     * @return The goal time interval.
     */
    [[nodiscard]] IntegerInterval getTime() const;

    /**
     * Getter for goal velocity interval.
     *
     * @return The goal velocity interval.
     */
    [[nodiscard]] std::optional<FloatInterval> getVelocity() const;

    /**
     * Getter for goal orientation interval.
     *
     * @return The goal orientation interval.
     */
    [[nodiscard]] std::optional<FloatInterval> getOrientation() const;

    /**
     * Getter for goal position lanelets.
     *
     * @return The goal position lanelets.
     */
    [[nodiscard]] std::vector<GoalPosition> getGoalPositions() const;

  private:
    IntegerInterval time;
    std::optional<FloatInterval> velocity;
    std::optional<FloatInterval> orientation;
    std::vector<GoalPosition> goalPositions;
};
