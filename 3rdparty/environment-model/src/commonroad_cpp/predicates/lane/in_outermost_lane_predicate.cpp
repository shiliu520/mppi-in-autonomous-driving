#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_outermost_lane_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/world.h>

bool InOutermostLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(
        lanelets.begin(), lanelets.end(), [additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
            auto adjacent{
                lanelet->getAdjacent(regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)))};
            return adjacent.adj == nullptr || adjacent.oppositeDir != adjacent.adj->getAdjacentRight().oppositeDir;
        });
}

double InOutermostLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                  const std::shared_ptr<Obstacle> &obstacleK,
                                                  const std::shared_ptr<Obstacle> &obstacleP,
                                                  const std::vector<std::string> &additionalFunctionParameters,
                                                  bool setBased) {
    throw std::runtime_error("InOutermostLanePredicate does not support robust evaluation!");
}

Constraint InOutermostLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    throw std::runtime_error("InOutermostLanePredicate does not support constraint evaluation!");
}

InOutermostLanePredicate::InOutermostLanePredicate() : CommonRoadPredicate(false) {}
