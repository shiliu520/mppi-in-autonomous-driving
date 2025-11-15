#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/velocity_below_threshold_predicate.h>

bool VelocityBelowThresholdPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters,
                                                        bool setBased) {
    return obstacleK->getVelocity(timeStep, setBased, true) <= stod(additionalFunctionParameters.at(0));
}

Constraint VelocityBelowThresholdPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("VelocityBelowThresholdPredicate does not support constraint evaluation!");
}

double VelocityBelowThresholdPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    throw std::runtime_error("VelocityBelowThresholdPredicate does not support robust evaluation!");
}

VelocityBelowThresholdPredicate::VelocityBelowThresholdPredicate() : CommonRoadPredicate(false) {}
