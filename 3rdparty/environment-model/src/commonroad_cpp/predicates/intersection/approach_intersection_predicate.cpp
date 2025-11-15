#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/intersection/approach_intersection_predicate.h>

bool ApproachIntersectionPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    const auto lanelets = obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    bool approach_intersection = false;
    for (const auto &lanelet : lanelets) {
        if (std::any_of(lanelet->getLaneletTypes().begin(), lanelet->getLaneletTypes().end(),
                        [](auto &laneletType) { return laneletType == LaneletType::intersection; }))
            return false;
        if (std::any_of(lanelet->getLaneletTypes().begin(), lanelet->getLaneletTypes().end(),
                        [](auto &laneletType) { return laneletType == LaneletType::incoming; }))
            approach_intersection = true;
    }

    return approach_intersection;
}
double ApproachIntersectionPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("ApproachIntersectionPredicate does not support robust evaluation!");
}
Constraint ApproachIntersectionPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("ApproachIntersectionPredicate does not support constraint evaluation!");
}
ApproachIntersectionPredicate::ApproachIntersectionPredicate() : CommonRoadPredicate(false) {}
