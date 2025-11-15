#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/regulatory/close_to_stop_line_predicate.h>
#include <commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.h>

bool CloseToStopLinePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    auto lanelets{obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep)};
    for (const auto &lanelet : lanelets) {
        auto stopLine{lanelet->getStopLine()};
        if (stopLine == nullptr)
            continue;

        if (obstacle_operations::lineInFrontOfObstacle(stopLine->getPoints(), obstacleK, timeStep,
                                                       world->getRoadNetwork()) and
            obstacle_operations::minDistanceToPoint(timeStep, stopLine->getPoints(), obstacleK) <
                parameters.getParam("closeStopLineDistance"))
            return true;
    }
    return false;
}

double CloseToStopLinePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    throw std::runtime_error("StopLineInFrontPredicate does not support robust evaluation!");
}

Constraint CloseToStopLinePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    throw std::runtime_error("StopLineInFrontPredicate does not support constraint evaluation!");
}

CloseToStopLinePredicate::CloseToStopLinePredicate() : CommonRoadPredicate(false) {}
