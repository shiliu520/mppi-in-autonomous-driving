#pragma once

#include "commonroad_cpp/auxiliaryDefs/structs.h"

/**
 * Class representing a signal state.
 */
class SignalState {
  public:
    /**
     * Default Constructor for a state without parameters. Everything state variable is set to false.
     */
    SignalState() = default;

    /**
     * Constructor initializing all variables.
     *
     * @param timeStep time step.
     * @param horn Boolean indicating activity of horn.
     * @param indicatorLeft Boolean indicating activity of left indicator.
     * @param indicatorRight Boolean indicating activity of right indicator.
     * @param brakingLights Boolean indicating activity of braking lights.
     * @param hazardWarningLights Boolean indicating activity of hazard warning lights.
     * @param flashingBlueLights Boolean indicating activity of flashing blue lights (police, ambulance).
     */
    SignalState(size_t timeStep, bool horn, bool indicatorLeft, bool indicatorRight, bool brakingLights,
                bool hazardWarningLights, bool flashingBlueLights);

    /**
     * Getter for time step.
     *
     * @return time step.
     */
    [[nodiscard]] size_t getTimeStep() const;

    /**
     * Getter for time step.
     *
     * @return time step.
     */

    [[nodiscard]] bool isHorn() const;

    /**
     * Getter for horn.
     *
     * @return Boolean indicating active horn.
     */

    /**
     * Getter for indicator left.
     *
     * @return Boolean indicating active indicator left.
     */

    [[nodiscard]] bool isIndicatorLeft() const;

    /**
     * Getter for indicator right.
     *
     * @return Boolean indicating active indicator right.
     */
    [[nodiscard]] bool isIndicatorRight() const;

    /**
     * Getter for braking lights.
     *
     * @return Boolean indicating active braking lights.
     */
    [[nodiscard]] bool isBrakingLights() const;

    /**
     * Getter for hazard warning lights.
     *
     * @return Boolean indicating active hazard warning lights.
     */
    [[nodiscard]] bool isHazardWarningLights() const;

    /**
     * Getter for flashing blue lights.
     *
     * @return Boolean indicating active flashing blue lights.
     */
    [[nodiscard]] bool isFlashingBlueLights() const;

    /**
     * Setter for horn.
     *
     * @param hornStatus Boolean indicating whether horn is activated.
     */
    void setHorn(bool hornStatus);

    /**
     * Setter for left indicator.
     *
     * @param inl Boolean indicating whether left indicator is activated.
     */
    void setIndicatorLeft(bool inl);

    /**
     * Setter for right indicator.
     *
     * @param inr Boolean indicating whether right indicator is activated.
     */
    void setIndicatorRight(bool inr);

    /**
     * Setter for braking lights.
     *
     * @param bls Boolean indicating whether braking lights are activated.
     */
    void setBrakingLights(bool bls);

    /**
     * Setter for hazard warning lights.
     *
     * @param hwl Boolean indicating whether hazard warning lights are activated.
     */
    void setHazardWarningLights(bool hwl);

    /**
     * Setter for flashing blue lights.
     *
     * @param fbl Boolean indicating whether flashing blue lights are activated.
     */
    void setFlashingBlueLights(bool fbl);

    /**
     * Setter for time step.
     *
     * @param tsp Time step of state.
     */
    void setTimeStep(size_t tsp);

    /**
     * Checks whether given signal is activated.
     *
     * @param signalName Name of signal.
     * @return Boolean indicating whether signal is activated.
     */
    bool isSignalSet(const std::string &signalName);

  private:
    bool horn{false};                //**< Boolean indicating activity of horn. */
    bool indicatorLeft{false};       //**< Boolean indicating activity of left indicator. */
    bool indicatorRight{false};      //**< Boolean indicating activity of right indicator. */
    bool brakingLights{false};       //**< Boolean indicating activity of braking lights. */
    bool hazardWarningLights{false}; //**< Boolean indicating activity of hazard warning lights. */
    bool flashingBlueLights{false};  //**< Boolean indicating activity of flashing blue lights (police, ambulance). */
    size_t timeStep{0};              //**< time step of the state variables */
};
