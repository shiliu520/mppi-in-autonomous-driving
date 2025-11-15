#include <iostream>

#include <commonroad_cpp/geometry/polygon.h>

void Polygon::printParameters() {
    std::cout << "--- Polygon Shape ---" << std::endl;
    for (const auto &vertex : this->getPolygon().outer())
        std::cout << "x: " << vertex.get<0>() << ", y: " << vertex.get<1>() << std::endl;
}

ShapeType Polygon::getType() { return ShapeType::polygon; }

Polygon::Polygon(const std::vector<vertex> &pol) {
    this->polygon.outer().resize(pol.size());
    size_t idx{0};
    for (const auto &left : pol) {
        this->polygon.outer()[idx] = point_type{left.x, left.y};
        idx++;
    }
}

[[nodiscard]] std::vector<vertex> Polygon::getPolygonVertices() const {
    std::vector<vertex> vertices;
    for (const auto &point : polygon.outer()) {
        vertices.emplace_back(vertex{point.x(), point.y()});
    }
    return vertices;
}
