#pragma once

/*
 * orientation/acceleration/velocity error
 * vehicle-specific: wheelbase, max steering angle
 * comfortable acceleration limit
 */

/**
 * ActuatorParameters includes all the static information required in order to predict
 * the motion of an obstacle, i.e., acceleration and velocity bounds but no dynamic
 * information such as current velocity or acceleration.
 *
 * ActuatorParameters + State + Global Parameters = Information required for Occupancy Prediction
 *
 * Note: The following assumptions about the quantites are enforced in the constructors:
 *   - \f$vMax \geq 0.0\f$
 *   - \f$aMax \geq 0.0\f$
 *   - \f$aMaxLong \geq 0.0\f$
 *   - \f$aMinLong \leq 0.0\f$
 *   - \f$aBraking \leq 0.0\f$
 *   - \f$aMaxLong \leq aMax\f$
 *   - \f$|aBraking| \leq |aMinLong|\f$
 *
 */
class ActuatorParameters {
    /** maximum velocity of obstacle in m/s */
    double vMax{50.0};
    /** maximum absolute acceleration of obstacle in [m/s^2] */
    double aMax{3.0};
    /** maximal longitudinal acceleration of obstacle in [m/s^2] */
    double aMaxLong{3.0};
    /** minimal longitudinal acceleration of obstacle in [m/s^2] */
    double aMinLong{-10.0};
    /** minimal (longitudinal) braking acceleration of obstacle in [m/s^2] */
    double aBraking{-10.0};

  public:
    /**
     * Default constructor.
     */
    ActuatorParameters() = default;

    /**
     * Complete constructor for ActuatorParameters.
     *
     * @param vMax maximum velocity of obstacle in [m/s]
     * @param aMax maximum absolute acceleration of obstacle in [m/s^2]
     * @param aMaxLong maximal longitudinal acceleration of obstacle in [m/s^2]
     * @param aMinLong minimal longitudinal acceleration of obstacle in [m/s^2]
     * @param aBraking minimal (longitudinal) braking acceleration of obstacle in [m/s^2]
     */
    ActuatorParameters(double vMax, double aMax, double aMaxLong, double aMinLong, double aBraking);

    /**
     * Simplified constructor for ActuatorParameters,
     * setting all acceleration limits based on aMax.
     *
     * @param vMax maximum velocity of obstacle in [m/s]
     * @param aMax maximum absolute acceleration of obstacle in [m/s^2]
     */
    ActuatorParameters(double vMax, double aMax);

    /**
     * Getter for maximum velocity the vehicle can drive.
     *
     * @return Maximum velocity [m/s].
     */
    [[nodiscard]] double getVmax() const noexcept;

    /**
     * Getter for maximum acceleration.
     *
     * @return Maximum acceleration [m/s^2].
     */
    [[nodiscard]] double getAmax() const noexcept;

    /**
     * Getter for maximum acceleration in longitudinal direction.
     *
     * @return Maximum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAmaxLong() const noexcept;

    /**
     * Getter for minimum acceleration in longitudinal direction.
     *
     * @return Minimum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAminLong() const noexcept;

    /**
     * Getter for minimum braking acceleration in longitudinal direction.
     *
     * @return Minimum acceleration in longitudinal direction [m/s^2].
     */
    [[nodiscard]] double getAbraking() const noexcept;

    /**
     * Default kinematic parameters for obstacle vehicles:
     * vMax = 50.0 m/s, aMax = 3.0 m/s^2, aMaxLong = 3 m/s^2, aMinLong = -10.5 m/s^2, aBraking = -10.5 m/s^2
     *
     *
     * @return Default kinematic parameters.
     */
    static ActuatorParameters vehicleDefaults();

    /**
     * Default kinematic parameters for ego vehicles:
     * vMax = 50.0 m/s, aMax = 3.0 m/s^2, aMaxLong = 3 m/s^2, aMinLong = -10.0 m/s^2, aBraking = -10.0 m/s^2
     *
     *
     * @return Default kinematic parameters.
     */
    static ActuatorParameters egoDefaults();

    /**
     * Default kinematic parameters for pedestrians:
     * vMax = 2.0 m/s, aMax = 0.6 m/s^2, other limits based on aMax,
     * reaction time = 0.3 s.
     *
     * @return Default pedestrian kinematic parameters.
     */
    static ActuatorParameters pedestrianDefaults();

    /**
     * Default kinematic parameters for static obstacles:
     * vMax = 0.0 m/s, aMax = 0.0 m/s^2.
     *
     * @return Default static obstacle kinematic parameters.
     */
    static ActuatorParameters staticDefaults();
};
