#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/right_of_predicate.h>

bool RightOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                         const std::shared_ptr<Obstacle> &obstacleK,
                                         const std::shared_ptr<Obstacle> &obstacleP,
                                         const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    auto rightObstacles{obstacle_operations::obstaclesRight(timeStep, {obstacleK}, obstacleP, world->getRoadNetwork())};
    return !rightObstacles.empty();
}

double RightOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                          const std::shared_ptr<Obstacle> &obstacleK,
                                          const std::shared_ptr<Obstacle> &obstacleP,
                                          const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    throw std::runtime_error("RightOfPredicate does not support robust evaluation!");
}

Constraint RightOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    throw std::runtime_error("RightOfPredicate does not support constraint evaluation!");
}

RightOfPredicate::RightOfPredicate() : CommonRoadPredicate(true) {}
