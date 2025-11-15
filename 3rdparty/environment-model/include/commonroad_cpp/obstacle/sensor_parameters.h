#pragma once
#include <optional>

#include <commonroad_cpp/auxiliaryDefs/structs.h>
#include <commonroad_cpp/geometry/types.h>

/**
 * SensorParameters includes information regarding obstacle sensors
 */
class SensorParameters {
    double fieldOfViewRear{100.0};                  //**< length of field of view provided by rear sensors */
    double fieldOfViewFront{150.0};                 //**< length of field of view provided by front sensors */
    std::optional<std::vector<vertex>> fovVertices; //**< field of view region */
    std::optional<polygon_type> fovPolygon;         //**< field of view as boost polygon */

  public:
    /**
     * Default constructor.
     */
    SensorParameters() = default;

    /**
     * Complete constructor for SensorParameters.
     *
     * @param fieldOfViewRear length of field of view provided by rear sensors
     * @param fieldOfViewFront length of field of view provided by front sensors
     */
    SensorParameters(double fieldOfViewRear, double fieldOfViewFront);

    /**
     * Getter for rear field of view.
     *
     * @return Rear field of view.
     */
    [[nodiscard]] double getFieldOfViewRear() const noexcept;

    /**
     * Getter for front field of view.
     *
     * @return Front field of view.
     */
    [[nodiscard]] double getFieldOfViewFront() const noexcept;

    /**
     * Getter for field of view vertices.
     * @return List of vertices spanning field of view.
     */
    [[nodiscard]] std::vector<vertex> getFieldOfViewVertices() const noexcept;

    /**
     * Getter for field of view polygon object.
     *
     * @return Boost polygon.
     */
    [[nodiscard]] polygon_type getFieldOfViewPolygon() const noexcept;

    /**
     * Setter of field of fiew.
     *
     * @param fovVertices Vertices representing field of view.
     */
    void setFov(const std::vector<vertex> &fovVertices);

    /**
     * Default sensor parameters for vehicles:
     * fieldOfViewRear = 100m, fieldOfViewFront = 150m.
     *
     *
     * @return Default dynamic obstacle sensor parameters.
     */
    static SensorParameters dynamicDefaults();

    /**
     * Default sensor parameters for static obstacles:
     * fieldOfViewRear = fieldOfViewFront = 0.0 m.
     *
     * @return Default static obstacle sensor parameters.
     */
    static SensorParameters staticDefaults();

    /**
     * Setter for default field of view.
     */
    void setDefaultFov();
};
