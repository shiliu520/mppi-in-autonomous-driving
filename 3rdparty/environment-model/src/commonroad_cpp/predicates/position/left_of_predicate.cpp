#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/position/left_of_predicate.h>

bool LeftOfPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                        const std::shared_ptr<Obstacle> &obstacleK,
                                        const std::shared_ptr<Obstacle> &obstacleP,
                                        const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    auto leftObstacles{obstacle_operations::obstaclesLeft(timeStep, {obstacleK}, obstacleP)};
    return !leftObstacles.empty();
}

double LeftOfPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                         const std::shared_ptr<Obstacle> &obstacleK,
                                         const std::shared_ptr<Obstacle> &obstacleP,
                                         const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    throw std::runtime_error("LeftOfPredicate does not support robust evaluation!");
}

Constraint LeftOfPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    throw std::runtime_error("LeftOfPredicate does not support constraint evaluation!");
}

LeftOfPredicate::LeftOfPredicate() : CommonRoadPredicate(true) {}
