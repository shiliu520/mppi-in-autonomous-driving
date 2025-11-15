#include "commonroad_cpp/obstacle/signal_state.h"
#include <algorithm>
#include <spdlog/spdlog.h>
#include <stdexcept>

size_t SignalState::getTimeStep() const { return timeStep; }

SignalState::SignalState(size_t timeStep, bool horn, bool indicatorLeft, bool indicatorRight, bool brakingLights,
                         bool hazardWarningLights, bool flashingBlueLights)
    : horn(horn), indicatorLeft(indicatorLeft), indicatorRight(indicatorRight), brakingLights(brakingLights),
      hazardWarningLights(hazardWarningLights), flashingBlueLights(flashingBlueLights), timeStep(timeStep) {}

bool SignalState::isHorn() const { return horn; }

bool SignalState::isIndicatorLeft() const { return indicatorLeft; }

bool SignalState::isIndicatorRight() const { return indicatorRight; }

bool SignalState::isBrakingLights() const { return brakingLights; }

bool SignalState::isHazardWarningLights() const { return hazardWarningLights; }

bool SignalState::isFlashingBlueLights() const { return flashingBlueLights; }

void SignalState::setHorn(bool hornStatus) { horn = hornStatus; }

void SignalState::setIndicatorLeft(bool inl) { indicatorLeft = inl; }

void SignalState::setIndicatorRight(bool inr) { indicatorRight = inr; }

void SignalState::setBrakingLights(bool bls) { brakingLights = bls; }

void SignalState::setHazardWarningLights(bool hwl) { hazardWarningLights = hwl; }

void SignalState::setFlashingBlueLights(bool fbl) { flashingBlueLights = fbl; }

void SignalState::setTimeStep(size_t tsp) { timeStep = tsp; }

bool SignalState::isSignalSet(const std::string &signalName) {
    auto sigNameTmp{signalName};
    std::transform(sigNameTmp.begin(), sigNameTmp.end(), sigNameTmp.begin(), ::tolower);
    if (sigNameTmp == "horn")
        return isHorn();
    if (sigNameTmp == "indicatorleft")
        return isIndicatorLeft();
    if (sigNameTmp == "indicatorright")
        return isIndicatorRight();
    if (sigNameTmp == "brakinglights")
        return isBrakingLights();
    if (sigNameTmp == "hazardwarninglights")
        return isHazardWarningLights();
    if (sigNameTmp == "flashingbluelights")
        return isFlashingBlueLights();
    spdlog::error("SignalState::isSignalSet: Unknown signal name!");
    return false;
}
