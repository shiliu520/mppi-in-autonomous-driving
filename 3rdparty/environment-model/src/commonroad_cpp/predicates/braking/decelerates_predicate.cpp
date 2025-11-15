#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/braking/decelerates_predicate.h>
#include <commonroad_cpp/world.h>

bool DeceleratesPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             const std::vector<std::string> &additionalFunctionParameters,
                                             bool setBased) {
    return robustEvaluation(timeStep, world, obstacleK, obstacleP, additionalFunctionParameters, setBased) <
           stod(additionalFunctionParameters.at(0));
}

Constraint DeceleratesPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    throw std::runtime_error("DeceleratesPredicate does not support constraint evaluation!");
}

double DeceleratesPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              bool setBased) {
    return obstacleK->getAcceleration(timeStep, setBased, true);
}

DeceleratesPredicate::DeceleratesPredicate() : CommonRoadPredicate(false) {}
