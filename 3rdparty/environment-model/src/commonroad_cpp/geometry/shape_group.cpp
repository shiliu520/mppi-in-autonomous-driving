#include <iostream>

#include <commonroad_cpp/geometry/shape_group.h>

void ShapeGroup::printParameters() {
    std::cout << "--- Shape Group ---" << std::endl;
    for (const auto &shape : shapeGroup)
        shape->printParameters();
}

ShapeType ShapeGroup::getType() { return ShapeType::shapeGroup; }

std::vector<std::shared_ptr<Shape>> &ShapeGroup::getShapes() { return shapeGroup; }

void ShapeGroup::addShape(const std::shared_ptr<Shape> &shape) { shapeGroup.push_back(shape); };
