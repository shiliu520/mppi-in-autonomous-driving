#include <commonroad_cpp/geometry/circle.h>

void Circle::setRadius(const double rad) { radius = rad; }

void Circle::setCenter(const double xPos, const double yPos) { center = vertex{xPos, yPos}; }

double Circle::getRadius() const { return radius; }

vertex Circle::getCenter() const { return center; }

double Circle::getCircumradius() const { return radius; }

void Circle::scaleShape(double factor) { this->setRadius(this->getRadius() * factor); }

ShapeType Circle::getType() { return ShapeType::circle; }
