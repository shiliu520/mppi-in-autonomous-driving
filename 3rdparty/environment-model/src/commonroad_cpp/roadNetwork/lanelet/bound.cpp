#include "commonroad_cpp/roadNetwork/lanelet/bound.h"

Bound::Bound(size_t boundaryId, const std::vector<vertex> &vertices, LineMarking lineMarking)
    : boundaryId(boundaryId), vertices(vertices), lineMarking(lineMarking) {}

size_t Bound::getBoundaryId() const { return boundaryId; }

const std::vector<vertex> &Bound::getVertices() const { return vertices; }

LineMarking Bound::getLineMarking() const { return lineMarking; }
