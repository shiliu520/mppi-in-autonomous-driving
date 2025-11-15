#include "commonroad_cpp/obstacle/obstacle_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/general/is_of_type_predicate.h>

IsOfTypePredicate::IsOfTypePredicate() : CommonRoadPredicate(false) {}

bool IsOfTypePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                          const std::shared_ptr<Obstacle> &obstacleK,
                                          const std::shared_ptr<Obstacle> &obstacleP,
                                          const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    std::string type{additionalFunctionParameters.at(0)};
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    if (type == "static")
        return obstacleK->isStatic();
    else if (type == "dynamic")
        return !obstacleK->isStatic();
    else if (obstacle_operations::matchStringToObstacleType(type) == ObstacleType::vru)
        return obstacleK->getObstacleType() == ObstacleType::bicycle or
               obstacleK->getObstacleType() == ObstacleType::pedestrian or
               obstacleK->getObstacleType() == ObstacleType::motorcycle;
    else if (obstacle_operations::matchStringToObstacleType(type) == ObstacleType::vehicle)
        return obstacleK->getObstacleType() == ObstacleType::car or
               obstacleK->getObstacleType() == ObstacleType::vehicle or
               obstacleK->getObstacleType() == ObstacleType::bus or
               obstacleK->getObstacleType() == ObstacleType::parked_vehicle or
               obstacleK->getObstacleType() == ObstacleType::priority_vehicle or
               obstacleK->getObstacleType() == ObstacleType::truck or
               obstacleK->getObstacleType() == ObstacleType::taxi or
               obstacleK->getObstacleType() == ObstacleType::bicycle;
    else if (obstacle_operations::matchStringToObstacleType(type) == ObstacleType::special_purpose_vehicle)
        return obstacleK->getObstacleType() == ObstacleType::bus or
               obstacleK->getObstacleType() == ObstacleType::taxi or
               obstacleK->getObstacleType() == ObstacleType::bicycle or
               obstacleK->getObstacleType() == ObstacleType::special_purpose_vehicle;
    else
        return obstacleK->getObstacleType() == obstacle_operations::matchStringToObstacleType(type);
}

double IsOfTypePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                           const std::shared_ptr<Obstacle> &obstacleK,
                                           const std::shared_ptr<Obstacle> &obstacleP,
                                           const std::vector<std::string> &additionalFunctionParameters,
                                           bool setBased) {
    throw std::runtime_error("IsOfTypePredicate does not support robust evaluation!");
}

Constraint IsOfTypePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    throw std::runtime_error("IsOfTypePredicate does not support constraint evaluation!");
}
