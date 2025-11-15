#include <commonroad_cpp/obstacle/occupancy.h>

#include "commonroad_cpp/auxiliaryDefs/types_and_definitions.h"
#include "commonroad_cpp/geometry/polygon.h"
#include "commonroad_cpp/geometry/shape_group.h"

Occupancy::Occupancy(size_t timeStep, std::shared_ptr<Shape> shape) : shape(shape), timeStep(timeStep) {}

std::shared_ptr<Shape> Occupancy::getShape() const { return shape; }

void Occupancy::setShape(std::shared_ptr<Shape> shape) { this->shape = shape; }

size_t Occupancy::getTimeStep() const { return timeStep; }

void Occupancy::setTimeStep(size_t time) { timeStep = time; }

multi_polygon_type Occupancy::getOccupancyPolygonShape() {
    multi_polygon_type mp;
    if (shape->getType() == ShapeType::polygon) {
        mp.push_back(std::dynamic_pointer_cast<Polygon>(this->shape)->getPolygon());
    } else if (this->shape->getType() == ShapeType::shapeGroup) {
        auto shapeGroup = std::dynamic_pointer_cast<ShapeGroup>(this->shape);
        for (const auto &shapeTmp : shapeGroup->getShapes())
            if (shapeTmp->getType() == ShapeType::polygon)
                mp.push_back(std::dynamic_pointer_cast<Polygon>(shapeTmp)->getPolygon());
            else
                throw std::runtime_error("Occupancy::getOccupancyPolygonShape: Only polygon shapes are supported.");
    }
    return mp;
}
