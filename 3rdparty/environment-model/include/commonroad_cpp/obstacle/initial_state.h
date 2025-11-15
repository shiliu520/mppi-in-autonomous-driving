#pragma once

#include <cstddef>

/**
 * Class representing a initial state.
 */
class InitialState {
  public:
    /**
     * Default Constructor without parameters for an initial state.
     */
    InitialState() = default;

    /**
     * Constructor initializing all variables.
     *
     * @param timeStep Time step.
     * @param xPosition X-Position.
     * @param yPosition Y-Position.
     * @param orientation Orientation.
     * @param velocity Velocity.
     * @param acceleration Acceleration.
     * @param yawRate Yaw rate.
     * @param slipAngle Slip angle.
     */
    InitialState(size_t timeStep, double xPosition, double yPosition, double orientation, double velocity,
                 double acceleration, double yawRate, double slipAngle);

    /**
     * Setter for x-position in Cartesian space.
     *
     * @param xPosition X-Position.
     */
    void setXPosition(double xPosition);

    /**
     * Setter for y-position in Cartesian space.
     *
     * @param yPosition Y-Position.
     */
    void setYPosition(double yPosition);

    /**
     * Setter for orientation.
     *
     * @param orientation Orientation.
     */
    void setOrientation(double orientation);

    /**
     * Setter for velocity.
     *
     * @param velocity Velocity.
     */
    void setVelocity(double velocity);

    /**
     * Setter for acceleration.
     *
     * @param acceleration Acceleration.
     */
    void setAcceleration(double acceleration);

    /**
     * Setter for yaw rate.
     *
     * @param yawRate Yaw rate.
     */
    void setYawRate(double yawRate);

    /**
     * Setter for slip angle.
     *
     * @param slipAngle Slip angle.
     */
    void setSlipAngle(double slipAngle);

    /**
     * Setter for time step.
     *
     * @param timeStep Time step.
     */
    void setTimeStep(size_t timeStep);

    /**
     * Getter for x-position in the Cartesian space.
     *
     * @return X-Position.
     */
    [[nodiscard]] double getXPosition() const;

    /**
     * Getter for y-position in the Cartesian space.
     *
     * @return Y-Position.
     */
    [[nodiscard]] double getYPosition() const;

    /**
     * Getter for orientation.
     *
     * @return Orientation.
     */
    [[nodiscard]] double getOrientation() const;

    /**
     * Getter for velocity.
     *
     * @return Velocity.
     */
    [[nodiscard]] double getVelocity() const;

    /**
     * Getter for acceleration.
     *
     * @return Acceleration.
     */
    [[nodiscard]] double getAcceleration() const;

    /**
     * Getter for yaw rate.
     *
     * @return Yaw rate.
     */
    [[nodiscard]] double getYawRate() const;

    /**
     * Getter for slip angle.
     *
     * @return Slip angle.
     */
    [[nodiscard]] double getSlipAngle() const;

    /**
     * Getter for time step.
     *
     * @return Time step.
     */
    [[nodiscard]] size_t getTimeStep() const;

  private:
    size_t timeStep{0};       //**< time step of the state variables */
    double xPosition{0.0};    //**< x-coordinate in Cartesian space [m] */
    double yPosition{0.0};    //**< y-coordinate in Cartesian space */
    double orientation{0.0};  //**< orientation in Cartesian space [rad] */
    double velocity{0.0};     //**< velocity [m/s] */
    double acceleration{0.0}; //**< acceleration [m/s^2] */
    double yawRate{0.0};      //**< yaw rate */
    double slipAngle{0.0};    //**< slip angle */
};
