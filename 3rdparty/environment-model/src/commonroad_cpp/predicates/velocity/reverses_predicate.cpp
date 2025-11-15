#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/reverses_predicate.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool ReversesPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                          const std::shared_ptr<Obstacle> &obstacleK,
                                          const std::shared_ptr<Obstacle> &obstacleP,
                                          const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    return obstacleK->getStateByTimeStep(timeStep)->getVelocity() < -parameters.getParam("standstillError");
}

Constraint ReversesPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    throw std::runtime_error("Reverses Predicate does not support constraint evaluation!");
}

double ReversesPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::vector<std::string> &additionalFunctionParameters,
                                           bool setBased) {
    throw std::runtime_error("Reverses Predicate does not support robust evaluation!");
}

ReversesPredicate::ReversesPredicate() : CommonRoadPredicate(false) {}
