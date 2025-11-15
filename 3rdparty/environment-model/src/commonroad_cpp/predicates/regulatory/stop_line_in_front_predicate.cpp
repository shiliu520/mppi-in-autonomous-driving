#include <Eigen/Dense>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

#include <commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.h>

bool StopLineInFrontPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep)};
    return std::any_of(lanelets.begin(), lanelets.end(), [&](const std::shared_ptr<Lanelet> &lanelet) {
        std::shared_ptr<StopLine> stopLine{lanelet->getStopLine()};
        if (stopLine == nullptr)
            return false;

        if (obstacle_operations::lineInFrontOfObstacle(stopLine->getPoints(), obstacleK, timeStep,
                                                       world->getRoadNetwork()) and
            obstacle_operations::minDistanceToPoint(timeStep, stopLine->getPoints(), obstacleK) <
                parameters.getParam("stopLineDistance"))
            return true;
        return false;
    });
}

double StopLineInFrontPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    throw std::runtime_error("StopLineInFrontPredicate does not support robust evaluation!");
}

Constraint StopLineInFrontPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    throw std::runtime_error("StopLineInFrontPredicate does not support constraint evaluation!");
}
StopLineInFrontPredicate::StopLineInFrontPredicate() : CommonRoadPredicate(false) {}
