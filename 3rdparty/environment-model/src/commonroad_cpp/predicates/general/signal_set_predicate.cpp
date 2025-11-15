#include "commonroad_cpp/predicates/general/signal_set_predicate.h"

#include <commonroad_cpp/obstacle/obstacle.h>

SignalSetPredicate::SignalSetPredicate() : CommonRoadPredicate(false) {}

bool SignalSetPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::vector<std::string> &additionalFunctionParameters,
                                           bool setBased) {
    return obstacleK->getSignalStateByTimeStep(timeStep)->isSignalSet(additionalFunctionParameters.at(0));
}

double SignalSetPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            bool setBased) {
    throw std::runtime_error("SignalSetPredicate does not support robust evaluation!");
}

Constraint SignalSetPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    throw std::runtime_error("SignalSetPredicate does not support constraint evaluation!");
}
