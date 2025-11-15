#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/lane/in_same_lane_predicate.h>

bool InSameLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            bool setBased) {
    for (const auto &laneK : obstacleK->getOccupiedLanesDrivingDirection(world->getRoadNetwork(), timeStep)) {
        const auto &relevantIDs{laneK->getContainedLaneletIDs()};
        for (const auto &laneletP : obstacleP->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep, setBased))
            if (relevantIDs.find(laneletP->getId()) != relevantIDs.end())
                return true;
    }
    return false;
}

double InSameLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             const std::vector<std::string> &additionalFunctionParameters,
                                             bool setBased) {
    throw std::runtime_error("In Same Lane Predicate does not support robust evaluation!");
}

Constraint InSameLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("In Same Lane Predicate does not support constraint evaluation!");
}
InSameLanePredicate::InSameLanePredicate() : CommonRoadPredicate(true) {}
