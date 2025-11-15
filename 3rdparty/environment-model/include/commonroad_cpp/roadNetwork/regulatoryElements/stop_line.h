#pragma once

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "traffic_light.h"
#include "traffic_sign.h"

/**
 * Class representing a CommonRoad stop line.
 */
class StopLine {
  public:
    /**
     * Constructor.
     *
     * @param points start and end vertex of stop line.
     * @param line_marking Line marking type of stop line.
     */
    StopLine(std::pair<vertex, vertex> points, LineMarking line_marking);

    /**
     * Default constructor.
     */
    StopLine() = default;

    /**
     * Getter for start and end point of stop line.
     *
     * @return pair of start and end vertex.
     */
    std::pair<vertex, vertex> getPoints();

    /**
     * Getter for line marking type of stop line.
     *
     * @return Line marking type.
     */
    [[nodiscard]] LineMarking getLineMarking() const;

    /**
     * Setter for start and end vertex of stop line.
     *
     * @param points Start and end vertex.
     */
    void setPoints(const std::pair<vertex, vertex> &position);

    /**
     * Setter for line marking of stop line.
     *
     * @param lineMarking Type of line marking.
     */
    void setLineMarking(LineMarking lineMarking);

  private:
    std::pair<vertex, vertex> points; //**< start and end vertex of stop line */
    LineMarking lineMarking;          //**< type of line marking */
};
