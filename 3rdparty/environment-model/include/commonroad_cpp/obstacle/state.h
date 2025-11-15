#pragma once

#include "commonroad_cpp/auxiliaryDefs/structs.h"

/**
 * Class representing a state.
 */
class State {
  public:
    /**
     * Default Constructor for a state without parameters. Everything state variable is set to
     * 0.0 and all validity values are set to false.
     */
    State() = default;

    /**
     * Constructor initializing all variables.
     *
     * @param timeStep time step.
     * @param xPosition x-position in Cartesian space.
     * @param yPosition y-position in Cartesian space.
     * @param velocity velocity in [m/s].
     * @param acceleration acceleration in [m/s^2].
     * @param globalOrientation Global orientation in [rad].
     * @param curvilinearOrientation Orientation along reference path in [rad].
     * @param lonPosition longitudinal position in Curvilinear domain.
     * @param latPosition lateral position in Curvilinear domain.
     */
    State(size_t timeStep, double xPosition, double yPosition, double velocity, double acceleration,
          double globalOrientation, double curvilinearOrientation, double lonPosition, double latPosition);

    /**
     * Constructor initializing all variables except longitudinal and lateral position.
     *
     * @param timeStep time step.
     * @param xPosition x-position in Cartesian space.
     * @param yPosition y-position in Cartesian space.
     * @param velocity velocity in [m/s].
     * @param acceleration acceleration in [m/s^2].
     * @param orientation orientation in [rad].
     *
     */
    State(size_t timeStep, double xPosition, double yPosition, double velocity, double acceleration,
          double orientation);

    /**
     * Getter for x-position.
     *
     * @return x-position of the state in the Cartesian space.
     */
    [[nodiscard]] double getXPosition() const;

    /**
     * Getter for x-position.
     *
     * @return x-position in Cartesian space.
     */
    [[nodiscard]] double getYPosition() const;

    /**
     * Getter for velocity.
     *
     * @return velocity.
     */
    [[nodiscard]] double getVelocity() const;

    /**
     * Getter for acceleration.
     *
     * @return acceleration.
     */
    [[nodiscard]] double getAcceleration() const;

    /**
     * Getter for longitudinal position.
     *
     * @return longitudinal position in Curvilinear domain.
     */
    [[nodiscard]] double getLonPosition() const;

    /**
     * Getter for lateral position.
     *
     * @return lateral position in Curvilinear domain.
     */
    [[nodiscard]] double getLatPosition() const;

    /**
     * Getter for global orientation.
     *
     * @return orientation.
     */
    [[nodiscard]] double getGlobalOrientation() const;

    /**
     * Getter for curvilinear orientation.
     *
     * @return orientation.
     */
    [[nodiscard]] double getCurvilinearOrientation() const;

    /**
     * Getter for time step.
     *
     * @return time step.
     */
    [[nodiscard]] size_t getTimeStep() const;

    /**
     * Getter for list of valid states elements.
     *
     * @return validity struct.
     */
    [[nodiscard]] const ValidStates &getValidStates() const;

    /**
     * Setter for x-position in Cartesian space.
     *
     * @param xPosition x-position in Cartesian space.
     */
    void setXPosition(double xPosition);

    /**
     * Setter for y-position in Cartesian space.
     *
     * @param yPosition y-position in Cartesian space.
     */
    void setYPosition(double yPosition);

    /**
     * Setter for velocity.
     *
     * @param velocity velocity in [m/s].
     */
    void setVelocity(double velocity);

    /**
     * Setter for acceleration.
     *
     * @param acceleration acceleration in [m/s^2].
     */
    void setAcceleration(double acceleration);

    /**
     * Setter for longitudinal position in Curvilinear domain.
     *
     * @param lonPosition longitudinal position in Curvilinear domain.
     */
    void setLonPosition(double lonPosition);

    /**
     * Setter for lateral position in Curvilinear domain.
     *
     * @param latPosition lateral position in Curvilinear domain.
     */
    void setLatPosition(double latPosition);

    /**
     * Setter for global orientation.
     *
     * @param orientation orientation in [rad].
     */
    void setGlobalOrientation(double orientation);

    /**
     * Setter for curvilinear orientation.
     *
     * @param orientation orientation in [rad].
     */
    void setCurvilinearOrientation(double orientation);

    /**
     * Setter for time step.
     *
     * @param timeStep time step.
     */
    void setTimeStep(size_t timeStep);

    /**
     * Getter for 2D vertex position.
     *
     * @return 2D Vertex.
     */
    vertex get2DVertex();

  private:
    double xPosition{0.0};                //**< x-coordinate in Cartesian space [m] */
    double yPosition{0.0};                //**< y-coordinate in Cartesian space */
    double velocity{0.0};                 //**< velocity [m/s] */
    double acceleration{0.0};             //**< acceleration [m/s^2] */
    double lonPosition{0.0};              //**< longitudinal position in curvilinear coordinate system [m] */
    double latPosition{0.0};              //**< lateral position in curvilinear coordinate system [m] */
    double globalOrientation{0.0};        //**< orientation in Cartesian space [rad] */
    double curvilinearOrientation{0.0};   //**< orientation along reference path [rad] */
    ValidStates validStates{false, false, //**< set of states which are already set and therefore are valid */
                            false, false, false, false, false, false};
    time_step_t timeStep{0}; //**< time step of the state variables */
};
