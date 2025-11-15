#include "commonroad_cpp/roadNetwork/road_network_config.h"

#include <cassert>

void RoadNetworkParameters::checkParameterValidity() const {
    assert(eps1 > 0.0);
    assert(eps2 > 0.0);
    assert(stepsToResamplePolyline > 0);
    assert(cornerCuttingRefinements > 0);
    assert(projectionDomainLimit > 0.0);
}
