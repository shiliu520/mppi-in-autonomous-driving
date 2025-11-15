#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>

#include "commonroad_cpp/predicates/lane/close_to_lane_border_predicate.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>

bool CloseToLaneBorderPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    std::vector<std::shared_ptr<Lane>> lanes{obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep)};
    if (regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)) == Direction::left)
        return std::all_of(lanes.begin(), lanes.end(), [obstacleK, this, timeStep](const std::shared_ptr<Lane> &lane) {
            return 0.5 * lane->getWidth(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                        obstacleK->getStateByTimeStep(timeStep)->getYPosition()) -
                       obstacleK->leftD(timeStep, lane->getCurvilinearCoordinateSystem()) <=
                   parameters.getParam("closeToLaneBorder");
        });
    else if (regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)) == Direction::right) {
        return std::all_of(lanes.begin(), lanes.end(), [obstacleK, this, timeStep](const std::shared_ptr<Lane> &lane) {
            return 0.5 * lane->getWidth(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                        obstacleK->getStateByTimeStep(timeStep)->getYPosition()) +
                       obstacleK->rightD(timeStep, lane->getCurvilinearCoordinateSystem()) <=
                   parameters.getParam("closeToLaneBorder");
        });
    }
    throw std::invalid_argument("CloseToLaneBorderPredicate::booleanEvaluation: Unknown side '" +
                                additionalFunctionParameters.at(0) + "'!");
}

double CloseToLaneBorderPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    throw std::runtime_error("CloseToLaneBorderPredicate does not support robust evaluation!");
}

Constraint CloseToLaneBorderPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("CloseToLaneBorderPredicate does not support constraint evaluation!");
}

CloseToLaneBorderPredicate::CloseToLaneBorderPredicate() : CommonRoadPredicate(false) {}
