#include "commonroad_cpp/roadNetwork/environment/environment.h"

Environment::Environment() {}

Environment::Environment(time_step_t timeStep, TimeOfDay timeOfDay, Weather weather, Underground underground)
    : timeStep(timeStep), timeOfDay(timeOfDay), weather(weather), underground(underground) {}
