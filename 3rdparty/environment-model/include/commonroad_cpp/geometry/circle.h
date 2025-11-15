#pragma once

#include "commonroad_cpp/auxiliaryDefs/structs.h"
#include "shape.h"

/**
 * Class representing a circle.
 */
class Circle : public Shape {
  public:
    /**
     * Constructor without parameters. Sets center to origin and radius to zero.
     */
    Circle() {
        radius = 0;
        center = vertex{0, 0};
    }

    /**
     * Constructor assigning only radius. Center is assigned the origin.
     *
     * @param rad Radius of circle [m].
     */
    explicit Circle(double rad) {
        radius = rad;
        center = {0.0, 0.0};
    }

    /**
     * Constructor assigning radius and center.
     *
     * @param rad Radius of circle [m].
     * @param vert Center vertex.
     */
    Circle(double rad, vertex vert) {
        radius = rad;
        center = vert;
    }

    /**
     * Setter for circle radius.
     *
     * @param rad Radius of circle [m].
     */
    void setRadius(double rad) override;

    /**
     * Setter for circle center.
     *
     * @param xPos x-coordinate of center [m].
     * @param yPos y-coordinate of center [m].
     */
    void setCenter(double xPos, double yPos) override;

    /**
     * Getter for radius of circle.
     *
     * @return Circle radius [m].
     */
    [[nodiscard]] double getRadius() const override;

    /**
     * Getter for type.
     *
     * @return Circle type.
     */
    ShapeType getType() override;

    /**
     * Getter for center circle. Function can only be used for circles.
     *
     * @return Center vertex.
     */
    [[nodiscard]] vertex getCenter() const override;

    /**
     * Getter for circle circumradius (for circles, this is equal to the radius).
     *
     * @return Circle circumradius.
     */
    [[nodiscard]] double getCircumradius() const override;

    /**
     * Function for scaling a circle.
     *
     * @param factor Scaling factor.
     */
    void scaleShape(double factor) override;

  private:
    double radius;   //**< Radius of shape. */
    vertex center{}; //**< Center of shape. */
};
