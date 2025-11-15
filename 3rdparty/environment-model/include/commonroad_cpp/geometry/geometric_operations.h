#pragma once

#include <vector>

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"

namespace geometric_operations {

/**
 * Add the dimensions of the object (rectangle with length and width) to the polygon vertices q in the object's
 * coordinate frame.
 *
 * @param qVertex Vertices for which the dimension should be adapted.
 * @param length Length to add.
 * @param width Width to add.
 * @return Vertices of rectangle polygon.
 */
std::vector<vertex> addObjectDimensionsRectangle(std::vector<vertex> qVertex, double length, double width);

/**
 * Add the dimensions of the object (circle with radius) to the polygon vertices q in the object's coordinate frame.
 * Note the polygon representation is just an approximation of the circle.
 *
 * @param qVertex Center vertex.
 * @param radius Radius to add.
 * @return Vertices of circle polygon.
 */
std::vector<vertex> addObjectDimensionsCircle(vertex qVertex, double radius);

/**
 *
 * @param xPos x-position of circle center.
 * @param yPos y-position of circle center.
 * @param aParam Parameter a specifying main primary axis of ellipse.
 * @param bParam Parameter b specifying secondary axis of ellipse.
 * @param resolution Resolution parameter for discretizing ellipse. Default value 64. Concrete discretization also
 * depends on aParam and bParam.
 * @return List of vertices representing circle.
 */
std::vector<vertex> discretizeEllipse(double xPos, double yPos, double aParam, double bParam, size_t resolution = 64);

/**
 * Rotate and translate the vertices from the special relative coordinates to the reference position and orientation
 * (transfer local coordinates to global coordinates)
 *
 * @param vertices Vertices which should be rotated and translated.
 * @param refPosition Translation factor.
 * @param refOrientation Rotation factor.
 * @return Vertices of shape.
 */
std::vector<vertex> rotateAndTranslateVertices(const std::vector<vertex> &vertices, vertex refPosition,
                                               double refOrientation);

/**
 * Computes orientation along polyline.
 *
 * @param polyline Polyline for which orientation should be calculated.
 * @return List of orientations along polyline.
 */
std::vector<double> computeOrientationFromPolyline(const std::vector<vertex> &polyline);

/**
 * Computes path length along polyline.
 *
 * @param polyline Polyline for which orientation should be calculated.
 * @return Path length along polyline.
 */
std::vector<double> computePathLengthFromPolyline(const std::vector<vertex> &polyline);

/**
 * Computes Euclidean distance in 2D space.
 *
 * @param pointA First vertex.
 * @param pointB Second vertex.
 * @return Distance.
 */
double euclideanDistance2Dim(const vertex &pointA, const vertex &pointB);

/**
 * Computes distance between each vertex of two polylines. The polylines must be of same length.
 *
 * @param polylineA First polyline.
 * @param polylineB Second polyline.
 * @return Width along polylines.
 */
std::tuple<std::vector<double>, double> computeDistanceFromPolylines(const std::vector<vertex> &polylineA,
                                                                     const std::vector<vertex> &polylineB);

/**
 * Interpolates value based on two polylines.
 * @param value Position where to evaluate value.
 * @param xValues x-coordinates of data points. Must be in ascending order.
 * @param yValues y-coordinates of data points.
 *
 * @return Interpolated value.
 */
double interpolate(double value, const std::vector<double> &xValues, const std::vector<double> &yValues);

/**
 * Return the signed difference between angles lhs and rhs
 *
 * @param lhs Left-hand side of the subtraction
 * @param rhs Right-hand side of the subtraction
 * @return  ``(lhs - rhs)``, the value will be within ``[-math.pi, math.pi)``.
    Both ``lhs`` and ``rhs`` may either be zero-based (within
    ``[0, 2*math.pi]``), or ``-pi``-based (within ``[-math.pi, math.pi]``).
 */
double subtractOrientations(double lhs, double rhs);

/**
 * Constrains angle to ``[-math.pi, math.pi]``.
 *
 * @param angle Angle to constrain [rad]
 * @return Constrained angle.
 */
double constrainAngle(double angle);

/**
 * Computes scalar product between two vertices.
 * @param vertA First vertex.
 * @param vertB Second vertex.
 * @return Scalar product.
 */
double scalarProduct(const vertex &vertA, const vertex &vertB);

/**
 * Compares two double values with threshold.
 *
 * @param fstValue First value
 * @param sndValue Second value
 * @param thresh Threshold
 * @return Equal or not
 */
bool equalValues(double fstValue, double sndValue, double thresh = std::numeric_limits<double>::epsilon());

/**
 * Gets the orientation of the lanelet in degree
 *
 * @param lanelet Lanelet
 * @return orientation in degree
 */
double getOrientationInDeg(std::shared_ptr<Lanelet> lanelet);

/**
 * Evaluates if two orientations in degree have a 90° to each other
 *
 * @param degree1 first orientation in degree
 * @param degree2 second orientation in degree
 * @return orientation in degree
 */
bool is90Deg(double degree1, double degree2);

/**
 * Evaluates if two orientations in degree have a 180° to each other
 *
 * @param degree1 first orientation in degree
 * @param degree2 second orientation in degree
 * @return orientation in degree
 */
bool is180Deg(double degree1, double degree2);

} // namespace geometric_operations
