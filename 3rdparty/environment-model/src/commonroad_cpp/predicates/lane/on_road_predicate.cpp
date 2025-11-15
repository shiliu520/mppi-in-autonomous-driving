#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/on_road_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
#include <commonroad_cpp/world.h>

OnRoadPredicate::OnRoadPredicate() : CommonRoadPredicate(false) {}

bool OnRoadPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                        const std::shared_ptr<Obstacle> &obstacleK,
                                        const std::shared_ptr<Obstacle> &obstacleP,
                                        const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    auto occ = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    return std::any_of(occ.begin(), occ.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
        return (!lanelet->hasLaneletType(LaneletType::sidewalk) and
                !lanelet->hasLaneletType(LaneletType::bicycleLane)) or
               lanelet_operations::bicycleLaneNextToRoad(lanelet);
    });
}

double OnRoadPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                         const std::shared_ptr<Obstacle> &obstacleK,
                                         const std::shared_ptr<Obstacle> &obstacleP,
                                         const std::vector<std::string> &additionalFunctionParameters, bool setBased) {
    throw std::runtime_error("OnRoadPredicate does not support robust evaluation!");
}

Constraint OnRoadPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                 const std::shared_ptr<Obstacle> &obstacleK,
                                                 const std::shared_ptr<Obstacle> &obstacleP,
                                                 const std::vector<std::string> &additionalFunctionParameters,
                                                 bool setBased) {
    throw std::runtime_error("OnRoadPredicate does not support constraint evaluation!");
}
