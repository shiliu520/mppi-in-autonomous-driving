#pragma once
#include <cstddef>
#include <memory>

#include "commonroad_cpp/geometry/types.h"

class Shape;

/**
 * Class representing an occupancy.
 */
class Occupancy {
  public:
    /**
     * Default Constructor for occupancy
     */
    Occupancy() = default;

    /**
     * Constructor initializing all variables.
     *
     * @param timeStep time step.
     * @param shape CommonRoad shape.
     */
    Occupancy(size_t timeStep, std::shared_ptr<Shape> shape);

    /**
     * Getter for shape.
     *
     * @return shape.
     */
    [[nodiscard]] std::shared_ptr<Shape> getShape() const;

    /**
     * Getter for time step.
     *
     * @return time step.
     */
    [[nodiscard]] size_t getTimeStep() const;

    /**
     * Setter for shape.
     *
     * @param shape CommonRoad shape.
     */
    void setShape(std::shared_ptr<Shape> shape);

    /**
     * Getter for occupancy as polygon shape.
     *
     * @return Occupancies as polygon shape.
     */
    multi_polygon_type getOccupancyPolygonShape();

    /**
     * Setter for time step.
     *
     * @param timeStep time step.
     */
    void setTimeStep(size_t timeStep);

  private:
    std::shared_ptr<Shape> shape; //**< shape of the occupancy */
    size_t timeStep{0};           //**< time step of the occupancy */
};
