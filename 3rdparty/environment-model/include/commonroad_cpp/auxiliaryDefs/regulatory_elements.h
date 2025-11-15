#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "regulatory_elements.h"
#include "types_and_definitions.h"

using TrafficSignTable = const std::unordered_map<TrafficSignTypes, std::string>;

extern TrafficSignTable TrafficSignIDGermany;
extern TrafficSignTable TrafficSignIDUSA;
extern TrafficSignTable TrafficSignIDSpain;
extern TrafficSignTable TrafficSignIDArgentina;
extern TrafficSignTable TrafficSignIDBelgium;
extern TrafficSignTable TrafficSignIDAustralia;

extern const std::unordered_map<std::string, TrafficSignTypes> TrafficSignNames;

extern const std::unordered_map<SupportedTrafficSignCountry, TrafficSignTable *> TrafficSignLookupTableByCountry;

extern const std::unordered_map<std::string, std::vector<int>> priorityTable;

extern const std::unordered_map<std::string, Direction> DirectionNames;

extern const std::unordered_map<std::string, TrafficLightState> TrafficLightStateNames;
