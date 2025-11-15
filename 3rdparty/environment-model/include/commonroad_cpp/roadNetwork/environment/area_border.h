#pragma once

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include <optional>

class AreaBorder {
  public:
    /**
     * Default Constructor
     */
    AreaBorder();

    /**
     * @param areaBorderID ID of area.
     * @param borderPoints Points of border.
     * @param adjacentID ID of adjacent area.
     * @param lineMarking Line marking of area border.
     */
    AreaBorder(size_t areaBorderID, std::vector<vertex> &borderPoints, std::optional<int> &adjacentID,
               LineMarking lineMarking);

  private:
    size_t areaBorderID{0};
    std::vector<vertex> borderPoints{};
    std::optional<int> adjacentID{};
    LineMarking lineMarking{LineMarking::unknown};
};
