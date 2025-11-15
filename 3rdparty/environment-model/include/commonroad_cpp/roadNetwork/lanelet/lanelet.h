#pragma once

#include <cstddef>
#include <memory>
#include <set>
#include <vector>

#include <commonroad_cpp/auxiliaryDefs/types_and_definitions.h>
#include <commonroad_cpp/geometry/types.h>
#include <commonroad_cpp/roadNetwork/types.h>

class Area;
class TrafficLight;
class TrafficSign;
class StopLine;
class Polygon;
struct vertex;
enum class ObstacleType;

/**
 * Class representing a lanelet.
 */
class Lanelet : public std::enable_shared_from_this<Lanelet> {
  public:
    /**
     * Default Constructor without parameters for a lanelet.
     */
    Lanelet() = default;

    /**
     * Constructor initializing borders, lanelet typ, and road users.
     * Creates automatically center vertices and polygon shape.
     *
     * @param laneletId ID of lanelet.
     * @param leftBorder Vector of vertices of left lanelet border.
     * @param rightBorder Vector of vertices of right lanelet border.
     * @param types List of types classifying lanelet.
     * @param usersOneWay List of road users one way.
     * @param usersBidirectional List of road users bidirectional.
     */
    Lanelet(size_t laneletId, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
            std::set<LaneletType> types, std::set<ObstacleType> usersOneWay = {},
            std::set<ObstacleType> usersBidirectional = {});

    /**
     * Constructor initializing borders, lanelet typ, road users, predecessor lanelets, and successor lanelets.
     * Creates automatically center vertices and polygon shape.
     *
     * @param laneletId ID of lanelet.
     * @param leftBorder Vector of vertices of left lanelet border.
     * @param rightBorder Vector of vertices of right lanelet border.
     * @param predecessorLanelets List of pointers to predecessor lanelets.
     * @param successorLanelets List of pointers to successor lanelets.
     * @param laneletTypes List of types classifying lanelet.
     * @param usersOneWay List of road users one way.
     * @param usersBidirectional List of road users bidirectional.
     */
    Lanelet(size_t laneletId, std::vector<vertex> leftBorder, std::vector<vertex> rightBorder,
            std::vector<std::shared_ptr<Lanelet>> predecessorLanelets,
            std::vector<std::shared_ptr<Lanelet>> successorLanelets, std::set<LaneletType> laneletTypes,
            std::set<ObstacleType> usersOneWay, std::set<ObstacleType> usersBidirectional);

    /*
     *   Adjacency struct containing a pointer to the adjacent lanelet and information about its driving direction.
     */
    struct adjacent {
        std::shared_ptr<Lanelet> adj;
        bool oppositeDir{false};
    };

    /**
     * Setter for ID of lanelet.
     *
     * @param lid ID of lanelet.
     */
    void setId(size_t lid);

    /**
     * Setter for adjacent left lanelet.
     *
     * @param left pointer to left lanelet.
     * @param oppositeDir driving direction of adjacent left lanelet.
     */
    void setLeftAdjacent(const std::shared_ptr<Lanelet> &left, bool oppositeDir);

    /**
     * Setter for adjacent right lanelet.
     *
     * @param right pointer to right lanelet.
     * @param oppositeDir driving direction of adjacent right lanelet.
     */
    void setRightAdjacent(const std::shared_ptr<Lanelet> &right, bool oppositeDir);

    /**
     * Setter for left lanelet border vertices.
     *
     * @param leftBorderVertices Vector of vertices of left lanelet border.
     */
    void setLeftBorderVertices(const std::vector<vertex> &leftBorderVertices);

    /**
     * Setter for right lanelet border vertices.
     *
     * @param rightBorderVertices Vector of vertices of right lanelet border.
     */
    void setRightBorderVertices(const std::vector<vertex> &rightBorderVertices);

    /**
     * Setter for lanelet type.
     *
     * @param laneletTypes list of types classifying lanelet.
     */
    void setLaneletTypes(const std::set<LaneletType> &laneletTypes);

    /**
     * Setter for road users one way.
     *
     * @param usersOneWay List of road users one way.
     */
    void setUsersOneWay(const std::set<ObstacleType> &usersOneWay);

    /**
     * Setter for road users bidirectional.
     *
     * @param usersBidirectional List of road users bidirectional.
     */
    void setUsersBidirectional(const std::set<ObstacleType> &usersBidirectional);

    /**
     * Setter for stop line.
     *
     * @param line Stop line belonging to lanelet.
     */
    void setStopLine(const std::shared_ptr<StopLine> &line);

    /**
     * Setter for left line marking.
     *
     * @param marking Left line marking.
     */
    void setLineMarkingLeft(LineMarking marking) const;

    /**
     * Setter for right line marking.
     *
     * @param marking Right line marking.
     */
    void setLineMarkingRight(LineMarking marking) const;

    /**
     * Appends a center vertex.
     *
     * @param center Vertex which should be appended.
     */
    void addCenterVertex(vertex center);

    /**
     * Appends a vertex to the left border.
     *
     * @param left Vertex which should be appended.
     */
    void addLeftVertex(vertex left);

    /**
     * Appends a vertex to the right border.
     *
     * @param right Vertex which should be appended.
     */
    void addRightVertex(vertex right);

    /**
     * Add a predecessor lanelet.
     *
     * @param pre Pointer to lanelet which should be added as predecessor.
     */
    void addPredecessor(const std::shared_ptr<Lanelet> &pre);

    /**
     * Add a successor lanelet.
     *
     * @param suc Pointer to lanelet which should be added as successor.
     */
    void addSuccessor(const std::shared_ptr<Lanelet> &suc);

    /**
     * Add a traffic light to lanelet.
     *
     * @param light Pointer to traffic light which should be added to lanelet.
     */
    void addTrafficLight(const std::shared_ptr<TrafficLight> &light);

    /**
     * Add a traffic sign to lanelet.
     *
     * @param sign Pointer to traffic sign which should be added to lanelet.
     */
    void addTrafficSign(const std::shared_ptr<TrafficSign> &sign);

    /**
     * Getter for lanelet ID.
     *
     * @return Lanelet ID.
     */
    [[nodiscard]] size_t getId() const;

    /**
     * Getter for predecessor lanelets.
     *
     * @return List of pointers to predecessor lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getPredecessors() const;

    /**
     * Getter for successor lanelets.
     *
     * @return List of pointers to successor lanelets.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<Lanelet>> &getSuccessors() const;

    /**
     * Getter for center vertices of lanelet.
     *
     * @return Vector containing center vertices.
     */
    [[nodiscard]] const std::vector<vertex> &getCenterVertices() const;

    /**
     * Getter for left border vertices of lanelet.
     *
     * @return Vector containing vertices of left border.
     */
    [[nodiscard]] const std::vector<vertex> &getLeftBorderVertices() const;

    /**
     * Getter for right border vertices of lanelet.
     *
     * @return Vector containing vertices of right border.
     */
    [[nodiscard]] const std::vector<vertex> &getRightBorderVertices() const;

    /**
     * Getter for traffic lights referenced to lanelet.
     *
     * @return List of pointers to traffic lights.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<TrafficLight>> &getTrafficLights() const;

    /**
     * Getter for traffic signs referenced to lanelet.
     *
     * @return List of pointers to traffic signs.
     */
    [[nodiscard]] const std::vector<std::shared_ptr<TrafficSign>> &getTrafficSigns() const;

    /**
     * Getter for polygon spanning the lanelet.
     *
     * @return Boost polygon.
     */
    [[nodiscard]] const polygon_type &getOuterPolygon() const;

    /**
     * Getter for bounding box of lanelet.
     *
     * @return Boost box.
     */
    [[nodiscard]] const box &getBoundingBox() const;

    /**
     * Getter for lanelet types.
     *
     * @return List of lanelet types.
     */
    [[nodiscard]] const std::set<LaneletType> &getLaneletTypes() const;

    /**
     * Getter for road users one way.
     *
     * @return List of road users one way.
     */
    [[nodiscard]] const std::set<ObstacleType> &getUsersOneWay() const;

    /**
     * Getter for road users bidirectional.
     *
     * @return List of road users bidirectional.
     */
    [[nodiscard]] const std::set<ObstacleType> &getUsersBidirectional() const;

    /**
     * Getter for adjacent left lanelet.
     *
     * @return Adjacent left lanelet struct.
     */
    [[nodiscard]] const adjacent &getAdjacentLeft() const;

    /**
     * Getter for adjacent right lanelet.
     *
     * @return Adjacent right lanelet struct.
     */
    [[nodiscard]] const adjacent &getAdjacentRight() const;

    /**
     * Getter for adjacent right/left lanelet.
     *
     * @param dir Side of lanelet.
     * @return Adjacent right/left lanelet struct.
     */
    [[nodiscard]] const adjacent &getAdjacent(Direction dir) const;

    /**
     * Getter for stop line assigned to lanelet.
     *
     * @return Stop line.
     */
    [[nodiscard]] const std::shared_ptr<StopLine> &getStopLine() const;

    /**
     * Getter for left line marking of lanelet.
     *
     * @return Left line marking.
     */
    [[nodiscard]] LineMarking getLineMarkingLeft() const;

    /**
     * Getter for right line marking of lanelet.
     *
     * @return Right line marking.
     */
    [[nodiscard]] LineMarking getLineMarkingRight() const;

    /**
     * Getter for right/left line marking of lanelet based on given direction.
     *
     * @param dir Side of lanelet.
     * @return Right line marking.
     */
    [[nodiscard]] LineMarking getLineMarking(Direction dir) const;

    /**
     * Given a polygon, checks whether the polygon intersects with the lanelet.
     *
     * @param polygon_shape boost polygon
     * @return boolean indicating whether lanelet is occupied
     */
    [[nodiscard]] bool applyIntersectionTesting(const polygon_type &polygon_shape) const;

    /**
     * Given a polygon, checks whether the polygon intersects with the lanelet given an intersection category.
     *
     * @param polygon_shape boost polygon
     * @param intersection_type specifies whether shape can be partially occupied by lanelet
     *  or must be completely occupied
     * @return boolean indicating whether lanelet is occupied
     */
    [[nodiscard]] bool checkIntersection(const polygon_type &polygon_shape, ContainmentType intersection_type) const;

    /**
     * Calculates center vertices as the arithmetic mean between the vertex on the left and right border.
     */
    void createCenterVertices();

    /**
     * Constructs boost polygon representing shape of lanelet based on left and right border vertices.
     */
    void constructOuterPolygon();

    /**
     * Computes orientation of lanelet given a x- and y-position of center vertices.
     *
     * @param positionX x-position of point
     * @param positionY y-position of point
     * @return orientation in interval [-pi, pi]
     */
    double getOrientationAtPosition(double positionX, double positionY) const;

    /**
     * Evaluates whether a lanelet has a specific lanelet type.
     *
     * @param laType Lanelet type which existence should be checked.
     * @return Boolean indicating whether the lanelet type exists.
     */
    bool hasLaneletType(LaneletType laType) const;

    /**
     * Evaluates whether a lanelet has several lanelet types.
     *
     * @param laType List of lanelet types which existence should be checked.
     * @return Boolean indicating whether the lanelet types match.
     */
    bool hasLaneletTypes(std::vector<LaneletType> laType) const;

    /**
     * Evaluates whether a lanelet has a specific traffic sign.
     *
     * @param trafficSignType trafficSign type which existence should be checked.
     * @return Boolean indicating whether the traffic sign exists.
     */
    bool hasTrafficSign(TrafficSignTypes trafficSignType) const;

    /**
     * Adds a lanelet type to the lanelet.
     *
     * @param laType Lanelet type which should be added.
     */
    void addLaneletType(LaneletType laType) const;

    /**
     * Computes list of orientation values along lanelet center line or returns already computed values.
     *
     * @return List of orientation values.
     */
    const std::vector<double> &getOrientation() const;

    /**
     * Computes list of path length values along lanelet center line or returns already computed values.
     *
     * @return List of path length values.
     */
    const std::vector<double> &getPathLength() const;

    /**
     * Computes list of distance/width values along vertices of left and right border
     * or returns already computed values.
     *
     * @return List of width values.
     */
    const std::vector<double> &getWidthAlongLanelet() const;

    /**
     * Computes width at x- and y-position along centerline.
     *
     * @param xPos x-position.
     * @param yPos y-position.
     * @return Width [m].
     */
    double getWidth(double xPos, double yPos) const;

    /**
     * Finds closest index on center line given 2D vertex. By default, function does not consider last index.
     *
     * @param positionX X-position of point of interest.
     * @param positionY Y-position of point of interest.
     * @param considerLastIndex Function also considers last index of lanelet.
     * @return Index of center line.
     */
    size_t findClosestIndex(double positionX, double positionY, bool considerLastIndex = false) const;

    /**
     * Computes minimum width of lanelet.
     *
     * @return Minimum width of lanelet.
     */
    double getMinWidth() const;

    /**
     * Computes intersection points of shape with lanelet.
     *
     * @param shape Multi polygon shape.
     * @return List of intersection vertices.
     */
    std::vector<vertex> computeIntersectionPointsWithShape(const multi_polygon_type &shape) const;

    /**
     *  Computes and sets adjacent lanelets.
     */
    void initAdjacentRoadLanes();

    /**
     *  Getter for adjacent left lanelets with the same driving direction.
     *
     * @return List of pointers to adjacent left lanelets with the same driving direction.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getAdjacentLeftSameDir() const;

    /**
     *  Getter for adjacent left lanelets.
     *
     * @return List of pointers to adjacent left lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getAdjacentLeftBothDir() const;

    /**
     * Getter for adjacent right lanelets with the same driving direction.
     *
     * @return List of pointers to adjacent right lanelets with the same driving direction.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getAdjacentRightSameDir() const;

    /**
     * Getter for adjacent right lanelets.
     *
     * @return List of pointers to adjacent right lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getAdjacentRightBothDir() const;

    /**
     * Getter for adjacent lanelets with the same driving direction.
     *
     * @return List of pointers to adjacent lanelets with the same driving direction.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getAdjacentSameDir() const;

    /**
     * Getter for adjacent lanelets.
     *
     * @return List of pointers to adjacent lanelets.
     */
    [[nodiscard]] std::vector<std::shared_ptr<Lanelet>> getAdjacentBothDir() const;

  private:
    mutable lanelet_id_t laneletId{};                                  //**< unique ID of lanelet */
    mutable std::vector<vertex> centerVertices;                        //**< vertices of center line of lanelet */
    mutable std::vector<vertex> leftBorder;                            //**< vertices of left border */
    mutable std::vector<vertex> rightBorder;                           //**< vertices of right border */
    mutable std::vector<std::shared_ptr<Lanelet>> predecessorLanelets; //**< list of pointers to predecessor lanelets */
    mutable std::vector<std::shared_ptr<Lanelet>> successorLanelets;   //**< list of pointers to successor lanelets */
    mutable adjacent
        adjacentLeft; //**< pointer to directly adjacent left lanelet and info about its driving direction */
    mutable adjacent
        adjacentRight; //**< pointer to directly adjacent right lanelet and info about its driving direction */
    mutable polygon_type outerPolygon; //**< Boost polygon of the lanelet */
    mutable box boundingBox{};         //**< Boost bounding box of the lanelet */
    mutable std::vector<std::shared_ptr<TrafficLight>>
        trafficLights; //**< list of pointers to traffic lights assigned to lanelet*/
    mutable std::vector<std::shared_ptr<TrafficSign>>
        trafficSigns; //**< list of pointers to traffic signs assigned to lanelet*/
    mutable std::vector<std::shared_ptr<Area>> adjacent_areas; //**< list of pointers to adjacent areas*/
    mutable std::set<LaneletType> laneletTypes;                //**< list of relevant lanelet types*/
    mutable std::set<ObstacleType> usersOneWay;                //**< list of relevant allowed users one way*/
    mutable std::set<ObstacleType> usersBidirectional;         //**< list of relevant allowed users bidirectional*/
    mutable std::shared_ptr<StopLine> stopLine;                //**< stopLine assigned to lanelet*/
    mutable LineMarking lineMarkingLeft;                       //**< Line marking of left boundary*/
    mutable LineMarking lineMarkingRight;                      //**< Line marking of right boundary*/
    mutable std::vector<double> orientation;                   //**< orientation along center line */
    mutable std::vector<double> pathLength;                    //**< path length along center line */
    mutable std::vector<double> width;                         //**< width along center line */
    mutable double minWidth;                                   //**< minimum width of lanelet */
    mutable std::vector<std::shared_ptr<Lanelet>>
        adjacentLeftSameDir; //**< all adjacent left lanelets with the same driving direction */
    mutable std::vector<std::shared_ptr<Lanelet>> adjacentLeftBothDir; //**< all adjacent left lanelets */
    mutable std::vector<std::shared_ptr<Lanelet>>
        adjacentRightSameDir; //**< all adjacent right lanelets with the same driving direction */
    mutable std::vector<std::shared_ptr<Lanelet>> adjacentRightBothDir; //**< all adjacent right lanelets */
    mutable std::vector<std::shared_ptr<Lanelet>>
        adjacentSameDir; //**< all adjacent lanelets with the same driving direction */
    mutable std::vector<std::shared_ptr<Lanelet>> adjacentBothDir; //**< all adjacent lanelets */
};

extern const std::unordered_map<std::string, LaneletType> LaneletTypeNames;

extern const std::unordered_map<std::string, bool> DrivingDirectionNames;
