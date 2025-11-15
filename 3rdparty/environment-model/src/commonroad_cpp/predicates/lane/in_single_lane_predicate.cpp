#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/in_single_lane_predicate.h>
#include <commonroad_cpp/world.h>

bool InSingleLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              bool setBased) {
    auto occLanelets{obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)};

    auto comparePreSuc = [](const std::vector<std::shared_ptr<Lanelet>> &lanelets1,
                            const std::vector<std::shared_ptr<Lanelet>> &lanelets2) {
        return lanelets1.size() == 1 and lanelets2.size() == 1 and lanelets1.at(0)->getId() == lanelets2.at(0)->getId();
    };

    return occLanelets.size() == 1 or
           (occLanelets.size() == 2 and !lanelet_operations::areLaneletsAdjacent(occLanelets[0], occLanelets[1])) or
           (occLanelets.size() == 2 and
            (comparePreSuc(occLanelets.at(0)->getPredecessors(), occLanelets.at(1)->getPredecessors()) or
             comparePreSuc(occLanelets.at(0)->getSuccessors(), occLanelets.at(1)->getSuccessors())));
}

double InSingleLanePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters,
                                               bool setBased) {
    throw std::runtime_error("InSingleLanePredicate does not support robust evaluation!");
}

Constraint InSingleLanePredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("InSingleLanePredicate does not support constraint evaluation!");
}
InSingleLanePredicate::InSingleLanePredicate() : CommonRoadPredicate(false) {}
