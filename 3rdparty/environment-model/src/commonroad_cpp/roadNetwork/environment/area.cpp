#include "commonroad_cpp/roadNetwork/environment/area.h"

Area::Area() = default;

Area::Area(size_t areaID, std::vector<AreaBorder> &borders, std::vector<AreaType> &areaTypes)
    : areaID(areaID), borders(borders), areaTypes(areaTypes) {}
