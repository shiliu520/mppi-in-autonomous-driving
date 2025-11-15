#include <commonroad_cpp/obstacle/state.h>
#include <stdexcept>

State::State(size_t timeStep, double xPosition, double yPosition, double velocity, double acceleration,
             double globalOrientation, double curvilinearOrientation, double lonPosition, double latPosition)
    : xPosition(xPosition), yPosition(yPosition), velocity(velocity), acceleration(acceleration),
      lonPosition(lonPosition), latPosition(latPosition), globalOrientation(globalOrientation),
      curvilinearOrientation(curvilinearOrientation),
      validStates(ValidStates{true, true, true, true, true, true, true, true}), timeStep(timeStep) {}

State::State(size_t timeStep, double xPosition, double yPosition, double velocity, double acceleration,
             double orientation)
    : xPosition(xPosition), yPosition(yPosition), velocity(velocity), acceleration(acceleration),
      globalOrientation(orientation), validStates(ValidStates{true, true, true, true, false, false, true, false}),
      timeStep(timeStep) {}

double State::getXPosition() const { return xPosition; }

void State::setXPosition(double xPos) {
    xPosition = xPos;
    validStates.xPosition = true;
}

double State::getYPosition() const { return yPosition; }

void State::setYPosition(double yPos) {
    yPosition = yPos;
    validStates.yPosition = true;
}

double State::getVelocity() const { return velocity; }

void State::setVelocity(double vel) {
    State::velocity = vel;
    validStates.velocity = true;
}

double State::getAcceleration() const {
    if (!validStates.acceleration)
        throw std::runtime_error("State::getAcceleration acceleration not initialized");
    return acceleration;
}

void State::setAcceleration(double acc) {
    acceleration = acc;
    validStates.acceleration = true;
}

double State::getLonPosition() const {
    if (!validStates.lonPosition)
        throw std::runtime_error("State::getLonPosition longitudinal position not initialized");
    return lonPosition;
}

void State::setLonPosition(double sPos) {
    lonPosition = sPos;
    validStates.lonPosition = true;
}

double State::getLatPosition() const {
    if (!validStates.latPosition)
        throw std::runtime_error("State::getLatPosition lateral position not initialized");
    return latPosition;
}

void State::setLatPosition(double dPos) {
    latPosition = dPos;
    validStates.latPosition = true;
}

double State::getGlobalOrientation() const { return globalOrientation; }

void State::setGlobalOrientation(double theta) {
    globalOrientation = theta;
    validStates.globalOrientation = true;
}

double State::getCurvilinearOrientation() const {
    if (!validStates.curvilinearOrientation)
        throw std::runtime_error("State::getCurvilinearOrientation curvilinear orientation not initialized");
    return curvilinearOrientation;
}

void State::setCurvilinearOrientation(double theta) {
    curvilinearOrientation = theta;
    validStates.curvilinearOrientation = true;
}

size_t State::getTimeStep() const { return timeStep; }

void State::setTimeStep(size_t time) { timeStep = time; }

const ValidStates &State::getValidStates() const { return validStates; }

vertex State::get2DVertex() { return {xPosition, yPosition}; }
