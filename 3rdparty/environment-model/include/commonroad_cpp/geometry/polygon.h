#pragma once

#include "shape.h"
#include <commonroad_cpp/geometry/geometric_operations.h>

#include <utility>

/**
 * Class representing a polygon.
 */
class Polygon : public Shape {
  public:
    /**
     * Constructor for polygon.
     *
     * @param polygon Polygon representation.
     */
    explicit Polygon(polygon_type polygon) : polygon(std::move(polygon)) {}

    /**
     * Constructor for polygon.
     *
     * @param pol Polygon representation as list of vertices.
     */
    explicit Polygon(const std::vector<vertex> &pol);

    /**
     * Getter for polygon.
     *
     * @return Polygon.
     */
    polygon_type getPolygon() { return polygon; }

    /**
     * Getter for polygon vertices.
     *
     * @return Polygon vertices.
     */
    [[nodiscard]] std::vector<vertex> getPolygonVertices() const;

    /**
     * Getter for type.
     *
     * @return Polygon shape type.
     */
    ShapeType getType() override;

    /**
     * Print function. Prints vertices on console output.
     */
    void printParameters() override;

  private:
    polygon_type polygon; // boost polygon representation
};
