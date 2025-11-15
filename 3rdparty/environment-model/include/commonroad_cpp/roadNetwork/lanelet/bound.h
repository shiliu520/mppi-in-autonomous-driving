#pragma once

#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>

class Bound {
  public:
    /**
     * Bound constructor.
     * @param boundaryId ID of boundary.
     * @param vertices Polyline of boundary.
     * @param lineMarking Line marking of boundary.
     */
    Bound(size_t boundaryId, const std::vector<vertex> &vertices, LineMarking lineMarking);

    /**
     * Getter of boundary ID.
     *
     * @return Boundary ID.
     */
    size_t getBoundaryId() const;

    /**
     * Getter for boundary vertices.
     *
     * @return Boundary polyline.
     */
    const std::vector<vertex> &getVertices() const;

    /**
     * Getter for boundary line marking.
     *
     * @return Line marking.
     */
    LineMarking getLineMarking() const;

  private:
    size_t boundaryId;
    std::vector<vertex> vertices;
    LineMarking lineMarking;
};
