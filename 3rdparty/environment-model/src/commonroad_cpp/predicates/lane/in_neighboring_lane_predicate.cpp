#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <unordered_set>

bool InNeighboringLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {
    std::unordered_set<unsigned long> relevantIDs;
    auto laneletsP = obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    for (const auto &laneK : obstacleK->getOccupiedRoadLanes(world->getRoadNetwork(), timeStep)) {
        for (const auto &laneletK : laneK->getContainedLanelets()) {
            auto adjacent{
                laneletK->getAdjacent(regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)))
                    .adj};
            if (adjacent and
                std::any_of(laneletsP.begin(), laneletsP.end(), [adjacent](const std::shared_ptr<Lanelet> &lanelet) {
                    return adjacent->getId() == lanelet->getId();
                })) {
                return true;
            }
        }
    }
    return false;
}

double InNeighboringLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    throw std::runtime_error("InNeighboringLanePredicate does not support robust evaluation!");
}

Constraint InNeighboringLanePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InNeighboringLanePredicate does not support constraint evaluation!");
}
InNeighboringLanePredicate::InNeighboringLanePredicate() : CommonRoadPredicate(true) {}
