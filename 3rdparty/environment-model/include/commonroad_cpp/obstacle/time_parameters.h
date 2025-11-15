#pragma once

#include <cstddef>

/**
 * TimeParameters includes information regarding obstacle time parameters
 */
class TimeParameters {
    size_t relevantHistorySize{50}; //**< number of history time steps to consider  */
    double reactionTime{0.3};       /** reaction time of obstacle in [s] */
    double timeStepSize{0.1};       /** time step size of obstacle in [s] */

  public:
    TimeParameters() = default;

    /**
     * Complete constructor for SensorParameters.
     *
     * @param relevantHistorySize length of history to consider
     * @param reactionTime reaction time of obstacle in [s]
     * @param timeStepSize time step size of obstacle in [s]
     */
    TimeParameters(size_t relevantHistorySize, double reactionTime, double timeStepSize);

    /**
     * Getter for relevant history.
     *
     * @return Size of relevant history.
     */
    [[nodiscard]] size_t getRelevantHistorySize() const noexcept;

    /**
     * Getter for reaction time.
     *
     * @return Reaction time [s].
     */
    [[nodiscard]] double getReactionTime() const noexcept;

    /**
     * Getter for time step size.
     *
     * @return Time step size [s].
     */
    [[nodiscard]] double getTimeStepSize() const noexcept;

    /**
     * Setter for time step size (time step size needs to be adjusted in world object -> therefore separate setter).
     *
     * @param timeStepSize New time step size [s].
     */
    void setTimeStepSize(double timeStepSize);

    /**
     * Default time parameters for dynamic obstacles: relevantHistorySize = 50, reactionTime = 0.3s.
     *
     * @return Default dynamic obstacle sensor parameters.
     */
    static TimeParameters dynamicDefaults();

    /**
     * Default time parameters for static obstacles: relevantHistorySize = 0, reactionTime = 0.0s.
     *
     * @return Default static obstacle sensor parameters.
     */
    static TimeParameters staticDefaults();
};
