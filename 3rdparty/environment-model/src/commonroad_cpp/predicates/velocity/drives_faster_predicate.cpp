#include "commonroad_cpp/predicates/lane/in_same_lane_predicate.h"
#include "commonroad_cpp/predicates/position/in_front_of_predicate.h"
#include "commonroad_cpp/predicates/velocity/in_standstill_predicate.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/velocity/drives_faster_predicate.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>
bool DrivesFasterPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              bool setBased) {
    return obstacleP->getVelocity(timeStep, setBased, false) < obstacleK->getVelocity(timeStep, setBased, true);
}

Constraint DrivesFasterPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("Drives Faster Predicate does not support constraint evaluation!");
}

double DrivesFasterPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters,
                                               bool setBased) {
    throw std::runtime_error("Drives Faster Predicate does not support robust evaluation!");
}

DrivesFasterPredicate::DrivesFasterPredicate() : CommonRoadPredicate(true) {}
