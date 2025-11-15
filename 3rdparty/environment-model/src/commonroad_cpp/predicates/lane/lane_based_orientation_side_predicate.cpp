#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/predicates/lane/lane_based_orientation_side_predicate.h>
#include <commonroad_cpp/world.h>

bool LaneBasedOrientationSidePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    return (regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)) == Direction::left and
            obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) > 0) or
           (regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)) == Direction::right and
            obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep) < 0);
}

double LaneBasedOrientationSidePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                           const std::shared_ptr<Obstacle> &obstacleK,
                                                           const std::shared_ptr<Obstacle> &obstacleP,
                                                           const std::vector<std::string> &additionalFunctionParameters,
                                                           bool setBased) {
    throw std::runtime_error("LaneBasedOrientationSidePredicate does not support robust evaluation!");
}

Constraint LaneBasedOrientationSidePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("LaneBasedOrientationSidePredicate does not support constraint evaluation!");
}
LaneBasedOrientationSidePredicate::LaneBasedOrientationSidePredicate() : CommonRoadPredicate(true) {}
