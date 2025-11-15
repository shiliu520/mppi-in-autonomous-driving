#pragma once

#include "area_border.h"

class Area {
  public:
    /**
     * Default Constructor
     */
    Area();

    /**
     * TODO
     * @param areaID
     * @param borders
     * @param areaTypes
     */
    Area(size_t areaID, std::vector<AreaBorder> &borders, std::vector<AreaType> &areaTypes);

  private:
    size_t areaID{0};
    std::vector<AreaBorder> borders;
    std::vector<AreaType> areaTypes;
};
