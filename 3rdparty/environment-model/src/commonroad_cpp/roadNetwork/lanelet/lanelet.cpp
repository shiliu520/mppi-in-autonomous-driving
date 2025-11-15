#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"

#include <commonroad_cpp/geometry/geometric_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <utility>

#include <limits>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/algorithms/unique.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <range/v3/all.hpp>

namespace bg = boost::geometry;

Lanelet::Lanelet(size_t laneletId, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
                 std::set<LaneletType> type, std::set<ObstacleType> oneWayUsers,
                 std::set<ObstacleType> bidirectionalUsers)
    : laneletId{laneletId}, leftBorder{std::move(leftBorder)}, rightBorder{std::move(rightBorder)},
      laneletTypes{std::move(type)}, usersOneWay{std::move(oneWayUsers)},
      usersBidirectional{std::move(bidirectionalUsers)} {
    createCenterVertices();
    constructOuterPolygon();
}

Lanelet::Lanelet(size_t laneletId, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
                 std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
                 std::vector<std::shared_ptr<Lanelet>> successorLanelets, std::set<LaneletType> type,
                 std::set<ObstacleType> oneWayUsers = {}, std::set<ObstacleType> bidirectionalUsers = {})
    : laneletId{laneletId}, leftBorder{std::move(leftBorder)}, rightBorder{std::move(rightBorder)},
      predecessorLanelets{std::move(predecessorLanelets)}, successorLanelets{std::move(successorLanelets)},
      laneletTypes{std::move(type)}, usersOneWay{std::move(oneWayUsers)},
      usersBidirectional{std::move(bidirectionalUsers)} {
    createCenterVertices();
    constructOuterPolygon();
}

void Lanelet::setId(const size_t lid) { laneletId = lid; }

void Lanelet::setLeftAdjacent(const std::shared_ptr<Lanelet> &left, bool oppositeDir) {
    adjacentLeft.adj = left;
    adjacentLeft.oppositeDir = oppositeDir;
}

void Lanelet::setRightAdjacent(const std::shared_ptr<Lanelet> &right, bool oppositeDir) {
    adjacentRight.adj = right;
    adjacentRight.oppositeDir = oppositeDir;
}

void Lanelet::setLeftBorderVertices(const std::vector<vertex> &leftBorderVertices) { leftBorder = leftBorderVertices; }

void Lanelet::setRightBorderVertices(const std::vector<vertex> &rightBorderVertices) {
    rightBorder = rightBorderVertices;
}

void Lanelet::setLaneletTypes(const std::set<LaneletType> &laType) { laneletTypes = laType; }

void Lanelet::setUsersOneWay(const std::set<ObstacleType> &user) { usersOneWay = user; }

void Lanelet::setUsersBidirectional(const std::set<ObstacleType> &user) { usersBidirectional = user; }

void Lanelet::setStopLine(const std::shared_ptr<StopLine> &line) { stopLine = line; }

void Lanelet::addLeftVertex(const vertex left) { leftBorder.push_back(left); }

void Lanelet::addRightVertex(const vertex right) { rightBorder.push_back(right); }

void Lanelet::addCenterVertex(const vertex center) { centerVertices.push_back(center); }

void Lanelet::addPredecessor(const std::shared_ptr<Lanelet> &pre) { predecessorLanelets.push_back(pre); }

void Lanelet::addSuccessor(const std::shared_ptr<Lanelet> &suc) { successorLanelets.push_back(suc); }

void Lanelet::addTrafficLight(const std::shared_ptr<TrafficLight> &light) { trafficLights.push_back(light); }

void Lanelet::addTrafficSign(const std::shared_ptr<TrafficSign> &sign) { trafficSigns.push_back(sign); }

size_t Lanelet::getId() const { return laneletId; }

const std::vector<std::shared_ptr<Lanelet>> &Lanelet::getPredecessors() const { return predecessorLanelets; }

const std::vector<std::shared_ptr<Lanelet>> &Lanelet::getSuccessors() const { return successorLanelets; }

const std::vector<vertex> &Lanelet::getCenterVertices() const { return centerVertices; }

const std::vector<vertex> &Lanelet::getLeftBorderVertices() const { return leftBorder; }

const std::vector<vertex> &Lanelet::getRightBorderVertices() const { return rightBorder; }

const std::vector<std::shared_ptr<TrafficLight>> &Lanelet::getTrafficLights() const { return trafficLights; }

const std::vector<std::shared_ptr<TrafficSign>> &Lanelet::getTrafficSigns() const { return trafficSigns; }

const polygon_type &Lanelet::getOuterPolygon() const { return outerPolygon; }

const box &Lanelet::getBoundingBox() const { return boundingBox; }

const std::set<LaneletType> &Lanelet::getLaneletTypes() const { return laneletTypes; }

const std::set<ObstacleType> &Lanelet::getUsersOneWay() const { return usersOneWay; }

const std::set<ObstacleType> &Lanelet::getUsersBidirectional() const { return usersBidirectional; }

const Lanelet::adjacent &Lanelet::getAdjacentLeft() const { return adjacentLeft; }

const Lanelet::adjacent &Lanelet::getAdjacentRight() const { return adjacentRight; }

const Lanelet::adjacent &Lanelet::getAdjacent(Direction dir) const {
    if (dir == Direction::left)
        return adjacentLeft;
    else if (dir == Direction::right)
        return adjacentRight;
    throw std::invalid_argument("Lanelet::adjacent: Invalid direction");
}

const std::shared_ptr<StopLine> &Lanelet::getStopLine() const { return stopLine; }

bool Lanelet::applyIntersectionTesting(const polygon_type &polygon_shape) const {
    // check first if shape intersects with bounding box since this evaluation is faster
    return bg::intersects(polygon_shape, this->getBoundingBox()) &&
           bg::intersects(polygon_shape, this->getOuterPolygon());
}

bool Lanelet::checkIntersection(const polygon_type &polygon_shape, ContainmentType intersection_type) const {
    switch (intersection_type) {
    case ContainmentType::PARTIALLY_CONTAINED: {
        return this->applyIntersectionTesting(polygon_shape);
    }
    case ContainmentType::COMPLETELY_CONTAINED: {
        return bg::within(polygon_shape, this->getOuterPolygon());
    }
    default:
        return false;
    }
}

std::vector<vertex> Lanelet::computeIntersectionPointsWithShape(const multi_polygon_type &shape) const {
    std::vector<vertex> intersectionPoints;
    multi_polygon_type output;

    // Compute the intersection between the lanelet polygon and the shape
    boost::geometry::intersection(outerPolygon, shape, output);

    // Extract intersection points from the resulting polygons
    for (const auto &polygon : output) {
        for (const auto &point : polygon.outer()) {
            vertex vert{point.x(), point.y()};
            intersectionPoints.emplace_back(vert);
        }
    }
    return intersectionPoints;
}

void Lanelet::initAdjacentRoadLanes() {
    adjacentLeftSameDir = lanelet_operations::laneletsLeftOfLanelet(shared_from_this(), true);
    adjacentLeftBothDir = lanelet_operations::laneletsLeftOfLanelet(shared_from_this(), false);
    adjacentRightSameDir = lanelet_operations::laneletsRightOfLanelet(shared_from_this(), true);
    adjacentRightBothDir = lanelet_operations::laneletsRightOfLanelet(shared_from_this(), false);
    adjacentBothDir =
        lanelet_operations::removeDuplicateLanelets(shared_from_this(), adjacentLeftBothDir, adjacentRightBothDir);
    adjacentSameDir =
        lanelet_operations::removeDuplicateLanelets(shared_from_this(), adjacentLeftSameDir, adjacentRightSameDir);
}

std::vector<std::shared_ptr<Lanelet>> Lanelet::getAdjacentLeftSameDir() const { return adjacentLeftSameDir; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getAdjacentLeftBothDir() const { return adjacentLeftBothDir; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getAdjacentRightSameDir() const { return adjacentRightSameDir; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getAdjacentRightBothDir() const { return adjacentRightBothDir; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getAdjacentSameDir() const { return adjacentSameDir; }

std::vector<std::shared_ptr<Lanelet>> Lanelet::getAdjacentBothDir() const { return adjacentBothDir; }

void Lanelet::constructOuterPolygon() {
    const std::vector<vertex> &leftBorderTemp = this->getLeftBorderVertices();
    const std::vector<vertex> &rightBorderTemp = this->getRightBorderVertices();

    if (!leftBorderTemp.empty()) {
        size_t idx = 0;
        polygon_type polygon;
        polygon.outer().resize(leftBorderTemp.size() + rightBorderTemp.size() + 1);

        for (const auto &left : leftBorderTemp) {
            polygon.outer()[idx] = point_type{left.x, left.y};
            idx++;
        }
        for (const auto &right : boost::adaptors::reverse(rightBorderTemp)) {
            polygon.outer()[idx] = point_type{right.x, right.y};
            idx++;
        }
        polygon.outer().back() = point_type{leftBorderTemp[0].x, leftBorderTemp[0].y};

        // Improve polygon (remove duplicated vertices, close vertices, order)
        bg::simplify(polygon, outerPolygon, 0.01);
        bg::unique(outerPolygon);
        bg::correct(outerPolygon);

        bg::envelope(outerPolygon, boundingBox); // set bounding box
    }
}

void Lanelet::createCenterVertices() {
    unsigned long numVertices = leftBorder.size();
    for (unsigned long i = 0; i < numVertices; i++) {
        vertex newVertex{};
        // calculate x and y values separately in order to minimize error
        newVertex.x = 0.5 * (leftBorder[i].x + rightBorder[i].x);
        newVertex.y = 0.5 * (leftBorder[i].y + rightBorder[i].y);
        addCenterVertex(newVertex);
    }
}

double Lanelet::getOrientationAtPosition(double positionX, double positionY) const {
    unsigned long closestIndex = findClosestIndex(positionX, positionY);

    // calculate orientation at vertex using its successor vertex
    vertex vert1{centerVertices[closestIndex]};
    vertex vert2{centerVertices[closestIndex + 1]};
    return atan2(vert2.y - vert1.y, vert2.x - vert1.x);
}

size_t Lanelet::findClosestIndex(double positionX, double positionY,
                                 bool considerLastIndex) const { // find the closest vertex to the given position
    assert(!centerVertices.empty());

    double minimum_diff = std::numeric_limits<double>::infinity();
    size_t minimum_index = 0;

    size_t numIterations{centerVertices.size() - 1};
    if (considerLastIndex)
        numIterations = centerVertices.size();

    for (size_t i = 0; i < numIterations; ++i) {
        double diffX = centerVertices[i].x - positionX;
        double diffY = centerVertices[i].y - positionY;

        // NOTE:
        // Instead of calculating sqrt(diffX * diffX + diffY + diffY), we leave out the square root here!
        // This is fine since we only calculate the distance in order to compare it with other distances,
        // and sqrt(a) <= sqrt(b) iff a <= b.
        // Therefore it is OK to use the squared distance, saving a few cycles of calculating the square root.
        // Since findClosestIndex() is called quite often depending on the use case, this optimization
        // can have a notable impact.
        double squared_diff = diffX * diffX + diffY * diffY;

        if (squared_diff < minimum_diff) {
            minimum_diff = squared_diff;
            minimum_index = i;
        }
    }

    // Sanity check: Is there any vertex closer than infinity?
    assert(minimum_diff != std::numeric_limits<double>::infinity());

    return minimum_index;
}

bool Lanelet::hasLaneletType(LaneletType laType) const {
    return laType == LaneletType::all or laType == LaneletType::any or laneletTypes.find(laType) != laneletTypes.end();
}

bool Lanelet::hasLaneletTypes(std::vector<LaneletType> laTypes) const {
    if (laTypes.size() == 1)
        return laTypes.at(0) == LaneletType::any or laneletTypes.find(laTypes.at(0)) != laneletTypes.end();
    return std::all_of(laTypes.begin(), laTypes.end(),
                       [this](const LaneletType ty) { return laneletTypes.find(ty) != laneletTypes.end(); });
}

bool Lanelet::hasTrafficSign(TrafficSignTypes trafficSignType) const {
    return std::any_of(trafficSigns.begin(), trafficSigns.end(),
                       [trafficSignType](const std::shared_ptr<TrafficSign> &ts) {
                           return !ts->getTrafficSignElementsOfType(trafficSignType).empty();
                       });
}

void Lanelet::addLaneletType(LaneletType laType) const { laneletTypes.insert(laType); }

LineMarking Lanelet::getLineMarkingLeft() const { return lineMarkingLeft; }

void Lanelet::setLineMarkingLeft(LineMarking marking) const { lineMarkingLeft = marking; }

LineMarking Lanelet::getLineMarkingRight() const { return lineMarkingRight; }

void Lanelet::setLineMarkingRight(const LineMarking marking) const { lineMarkingRight = marking; }

const std::vector<double> &Lanelet::getOrientation() const {
    if (orientation.empty())
        orientation = geometric_operations::computeOrientationFromPolyline(centerVertices);
    return orientation;
}

const std::vector<double> &Lanelet::getPathLength() const {
    if (pathLength.empty())
        pathLength = geometric_operations::computePathLengthFromPolyline(centerVertices);
    return pathLength;
}

const std::vector<double> &Lanelet::getWidthAlongLanelet() const {
    if (width.empty())
        width = std::get<0>(geometric_operations::computeDistanceFromPolylines(leftBorder, rightBorder));
    return width;
}

double Lanelet::getWidth(double positionX, double positionY) const {
    if (width.empty()) {
        auto tmpWidth{geometric_operations::computeDistanceFromPolylines(leftBorder, rightBorder)};
        width = std::get<0>(tmpWidth);
        minWidth = std::get<1>(tmpWidth);
    }
    unsigned long closestIndex = findClosestIndex(positionX, positionY);
    vertex vertS{centerVertices[closestIndex + 1] - centerVertices[closestIndex]};
    vertex vertV{vertex{positionX, positionY} - centerVertices[closestIndex]};
    vertex vertP{vertS * (geometric_operations::scalarProduct(vertS, vertV) /
                          geometric_operations::scalarProduct(vertS, vertS))};
    double scalar{
        geometric_operations::euclideanDistance2Dim(centerVertices[closestIndex],
                                                    vertP + centerVertices[closestIndex]) /
        geometric_operations::euclideanDistance2Dim(centerVertices[closestIndex], centerVertices[closestIndex + 1])};
    return width[closestIndex] + (width[closestIndex + 1] - width[closestIndex]) * scalar;
}

double Lanelet::getMinWidth() const {
    if (width.empty()) {
        auto tmpWidth{geometric_operations::computeDistanceFromPolylines(leftBorder, rightBorder)};
        width = std::get<0>(tmpWidth);
        minWidth = std::get<1>(tmpWidth);
    }
    return minWidth;
}

LineMarking Lanelet::getLineMarking(Direction dir) const {
    if (dir == Direction::left)
        return lineMarkingLeft;
    else if (dir == Direction::right)
        return lineMarkingRight;
    throw std::invalid_argument("Lanelet::getLineMarking: Invalid direction");
}

const std::unordered_map<std::string, LaneletType> LaneletTypeNames = {
    {"INTERSTATE", LaneletType::interstate},
    {"URBAN", LaneletType::urban},
    {"CROSSWALK", LaneletType::crosswalk},
    {"BUSSTOP", LaneletType::busStop},
    {"COUNTRY", LaneletType::country},
    {"HIGHWAY", LaneletType::highway},
    {"DRIVEWAY", LaneletType::driveWay},
    {"MAINCARRIAGEWAY", LaneletType::mainCarriageWay},
    {"ACCESSRAMP", LaneletType::accessRamp},
    {"EXITRAMP", LaneletType::exitRamp},
    {"SHOULDER", LaneletType::shoulder},
    {"BUSLANE", LaneletType::busLane},
    {"BICYCLELANE", LaneletType::bicycleLane},
    {"SIDEWALK", LaneletType::sidewalk},
    {"UNKNOWN", LaneletType::unknown},
    {"INTERSECTION", LaneletType::intersection},
    {"LEFT", LaneletType::left},
    {"RIGHT", LaneletType::right},
    {"STRAIGHT", LaneletType::straight},
    {"INTERSECTIONLEFTOUTGOING", LaneletType::intersectionLeftOutgoing},
    {"INTERSECTIONRIGHTOUTGOING", LaneletType::intersectionRightOutgoing},
    {"INTERSECTIONSTRAIGHTOUTGOING", LaneletType::intersectionStraightOutgoing},
    {"INCOMING", LaneletType::incoming},
    {"BORDER", LaneletType::border},
    {"PARKING", LaneletType::parking},
    {"RESTRICTED", LaneletType::restricted},
    {"RESTRICTEDAREA", LaneletType::restrictedArea},
    {"ANY", LaneletType::any},
    {"ALL", LaneletType::all},
};
