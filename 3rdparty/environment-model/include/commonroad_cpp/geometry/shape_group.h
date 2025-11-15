#pragma once

#include "shape.h"
#include <commonroad_cpp/geometry/geometric_operations.h>

/**
 * Class representing a shape group.
 */
class ShapeGroup : public Shape {
  public:
    /**
     * Default constructor.
     */
    ShapeGroup() = default;

    /**
     * Constructor for shape group.
     *
     * @param shapeGroup List of shapes.
     */
    explicit ShapeGroup(const std::vector<std::shared_ptr<Shape>> &shapeGroup) : shapeGroup(shapeGroup) {}

    /**
     * Getter for shape group.
     *
     * @return List of shapes.
     */
    std::vector<std::shared_ptr<Shape>> &getShapes();

    /**
     * Adds shape to shape group.
     *
     * @param shape Shape to add.
     */
    void addShape(const std::shared_ptr<Shape> &shape);

    /**
     * Getter for type.
     *
     * @return Shape group shape type.
     */
    ShapeType getType() override;

    /**
     * Print function. Prints each individual group element on console output.
     */
    void printParameters() override;

  private:
    std::vector<std::shared_ptr<Shape>> shapeGroup; // shape group
};
