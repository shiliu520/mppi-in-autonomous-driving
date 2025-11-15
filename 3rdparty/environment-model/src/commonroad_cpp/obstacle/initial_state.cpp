#include <commonroad_cpp/obstacle/initial_state.h>

InitialState::InitialState(size_t timeStep, double xPosition, double yPosition, double orientation, double velocity,
                           double acceleration, double yawRate, double slipAngle)
    : timeStep(timeStep), xPosition(xPosition), yPosition(yPosition), orientation(orientation), velocity(velocity),
      acceleration(acceleration), yawRate(yawRate), slipAngle(slipAngle) {}

void InitialState::setXPosition(double xPos) { xPosition = xPos; }

void InitialState::setYPosition(double yPos) { yPosition = yPos; }

void InitialState::setOrientation(double orie) { orientation = orie; }

void InitialState::setVelocity(double velo) { velocity = velo; }

void InitialState::setAcceleration(double acc) { acceleration = acc; }

void InitialState::setYawRate(double yaw) { yawRate = yaw; }

void InitialState::setSlipAngle(double slip) { slipAngle = slip; }

void InitialState::setTimeStep(size_t time) { timeStep = time; }

double InitialState::getXPosition() const { return xPosition; }

double InitialState::getYPosition() const { return yPosition; }

double InitialState::getOrientation() const { return orientation; }

double InitialState::getVelocity() const { return velocity; }

double InitialState::getAcceleration() const { return acceleration; }

double InitialState::getYawRate() const { return yawRate; }

double InitialState::getSlipAngle() const { return slipAngle; }

size_t InitialState::getTimeStep() const { return timeStep; }
