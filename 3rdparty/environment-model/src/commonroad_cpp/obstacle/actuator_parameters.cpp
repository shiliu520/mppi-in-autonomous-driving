#include "commonroad_cpp/obstacle/actuator_parameters.h"
#include <cassert>
#include <cmath>

ActuatorParameters::ActuatorParameters(double vMax, double aMax, double aMaxLong, double aMinLong, double aBraking)
    : vMax{vMax}, aMax{aMax}, aMaxLong{aMaxLong}, aMinLong{aMinLong}, aBraking{aBraking} {

    assert(vMax >= 0.0);
    assert(aMax >= 0.0);
    assert(aMaxLong >= 0.0);
    assert(aMinLong <= 0.0);
    assert(aBraking <= 0.0);
    assert(aMaxLong <= aMax);
    assert(std::abs(aBraking) <= std::abs(aMinLong));
}

ActuatorParameters::ActuatorParameters(double vMax, double aMax) : ActuatorParameters{vMax, aMax, aMax, -aMax, -aMax} {}

double ActuatorParameters::getVmax() const noexcept { return vMax; }

double ActuatorParameters::getAmax() const noexcept { return aMax; }

double ActuatorParameters::getAmaxLong() const noexcept { return aMaxLong; }

double ActuatorParameters::getAminLong() const noexcept { return aMinLong; }

double ActuatorParameters::getAbraking() const noexcept { return aBraking; }

ActuatorParameters ActuatorParameters::vehicleDefaults() { return ActuatorParameters{50.0, 5.0, 5.0, -10.5, -10.5}; }

ActuatorParameters ActuatorParameters::egoDefaults() { return ActuatorParameters{50.0, 3.0, 3.0, -10.0, -10.0}; }

ActuatorParameters ActuatorParameters::pedestrianDefaults() { return ActuatorParameters{2.0, 0.6}; }

ActuatorParameters ActuatorParameters::staticDefaults() { return ActuatorParameters{0.0, 0.0}; }
