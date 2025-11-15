#pragma once

#include "commonroad_cpp/auxiliaryDefs/structs.h"

/**
 * Class representing a shape.
 */
class Shape {
  public:
    virtual ~Shape() = default;

    /**
     * Virtual setter for shape length. Function can only be used for rectangles.
     *
     * @param l Length of shape [m].
     */
    virtual void setLength(const double l) {};

    /**
     * Virtual setter for shape width. Function can only be used for rectangles.
     *
     * @param w Width of shape [m].
     */
    virtual void setWidth(const double w) {};

    /**
     * Virtual setter for shape radius. Function can only be used for circles.
     *
     * @param rad Radius of shape [m].
     */
    virtual void setRadius(const double rad) {};

    /**
     * Virtual setter for shape center. Function can only be used for circles.
     *
     * @param x x-coordinate of center [m].
     * @param y y-coordinate of center [m].
     */
    virtual void setCenter(const double x, const double y) {};

    /**
     * Virtual getter for shape length. Function cannot be used for circles.
     *
     * @return Shape length [m].
     */
    [[nodiscard]] virtual double getLength() const { return 0.0; };

    /**
     * Virtual getter for shape width. Function cannot be used for circles.
     *
     * @return Shape width [m].
     */
    [[nodiscard]] virtual double getWidth() const { return 0.0; };

    /**
     * Virtual getter for shape radius. Function can only be used for circles.
     *
     * @return Shape radius [m].
     */
    [[nodiscard]] virtual double getRadius() const { return 0.0; };

    /**
     * Virtual getter for shape circumradius (radius of circumscribed circle).
     *
     * @return Shape circumradius [m].
     */
    [[nodiscard]] virtual double getCircumradius() const { return 0.0; };

    /**
     * Virtual getter for shape center. Function can only be used for circles.
     *
     * @return Shape center.
     */
    [[nodiscard]] virtual vertex getCenter() const { return {0, 0}; };

    /**
     * Virtual getter for shape type.
     *
     * @return Shape type, e.g. circle or rectangle.
     */
    [[nodiscard]] virtual ShapeType getType() = 0;

    /**
     * Virtual function for scaling a shape.
     *
     * @param factor Scaling factor.
     */
    virtual void scaleShape(double factor) {}

    /**
     * Virtual print function. Prints general information about shape.
     */
    virtual void printParameters() {};
};
