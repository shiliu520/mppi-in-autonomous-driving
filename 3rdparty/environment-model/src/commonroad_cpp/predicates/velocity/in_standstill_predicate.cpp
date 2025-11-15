#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/in_standstill_predicate.h>

bool InStandstillPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              bool setBased) {
    return -parameters.getParam("standstillError") < obstacleK->getVelocity(timeStep, setBased, true) and
           parameters.getParam("standstillError") > obstacleK->getVelocity(timeStep, setBased, true);
}

double InStandstillPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters,
                                               bool setBased) {
    throw std::runtime_error("InStandstillPredicate does not support robust evaluation!");
}

Constraint InStandstillPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("InStandstillPredicate does not support constraint evaluation!");
}
InStandstillPredicate::InStandstillPredicate() : CommonRoadPredicate(false) {}
