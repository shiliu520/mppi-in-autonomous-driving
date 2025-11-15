#include <iostream>

#include <commonroad_cpp/geometry/rectangle.h>

void Rectangle::setLength(const double len) { length = len; }

void Rectangle::setWidth(const double wid) { width = wid; }

double Rectangle::getLength() const { return length; }

double Rectangle::getWidth() const { return width; }

double Rectangle::getCircumradius() const {
    double diagonal = std::hypot(length, width);
    return diagonal / 2.0;
}

void Rectangle::scaleShape(double factor) {
    this->setWidth(this->getWidth() * factor);
    this->setLength(this->getLength() * factor);
}

void Rectangle::printParameters() {
    std::cout << "--- Rectangle Shape ---" << std::endl;
    std::cout << "length: " << this->getLength() << std::endl;
    std::cout << "width: " << this->getWidth() << std::endl;
}

ShapeType Rectangle::getType() { return ShapeType::rectangle; }
