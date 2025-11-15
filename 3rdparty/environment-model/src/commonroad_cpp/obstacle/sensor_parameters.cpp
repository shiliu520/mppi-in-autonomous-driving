#include "commonroad_cpp/obstacle/sensor_parameters.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <commonroad_cpp/auxiliaryDefs/structs.h>

class vertex;

SensorParameters::SensorParameters(double fieldOfViewRear, double fieldOfViewFront)
    : fieldOfViewRear{fieldOfViewRear}, fieldOfViewFront{fieldOfViewFront} {}

double SensorParameters::getFieldOfViewRear() const noexcept { return fieldOfViewRear; }

double SensorParameters::getFieldOfViewFront() const noexcept { return fieldOfViewFront; }

std::vector<vertex> SensorParameters::getFieldOfViewVertices() const noexcept { return fovVertices.value(); }

polygon_type SensorParameters::getFieldOfViewPolygon() const noexcept { return fovPolygon.value(); }

void SensorParameters::setFov(const std::vector<vertex> &fovVertices) {
    this->fovVertices = fovVertices;
    polygon_type polygon;
    polygon.outer().resize(fovVertices.size());
    size_t idx{0};
    for (const auto &left : fovVertices) {
        polygon.outer()[idx] = point_type{left.x, left.y};
        idx++;
    }

    fovPolygon = polygon;
    boost::geometry::simplify(polygon, fovPolygon.value(), 0.01);
    boost::geometry::unique(fovPolygon.value());
    boost::geometry::correct(fovPolygon.value());
}

SensorParameters SensorParameters::dynamicDefaults() { return SensorParameters{100.0, 150.0}; }

SensorParameters SensorParameters::staticDefaults() { return SensorParameters{0.0, 0.0}; }

void SensorParameters::setDefaultFov() {
    setFov({{0.0, -400.0},
            {282.8427, -282.8427},
            {400.0, 0.0},
            {282.8427, 282.8427},
            {0.0, 400.0},
            {-282.8427, 282.8427},
            {-400.0, 0.0},
            {-282.8427, -282.8427},
            {0.0, -400.0}});
}
