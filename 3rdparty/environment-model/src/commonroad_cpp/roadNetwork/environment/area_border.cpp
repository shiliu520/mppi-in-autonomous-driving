#include "commonroad_cpp/roadNetwork/environment/area_border.h"

AreaBorder::AreaBorder() = default;

AreaBorder::AreaBorder(size_t areaBorderID, std::vector<vertex> &borderPoints, std::optional<int> &adjacentID,
                       LineMarking lineMarking)
    : areaBorderID(areaBorderID), borderPoints(borderPoints), adjacentID(adjacentID), lineMarking(lineMarking) {}
