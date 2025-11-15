#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>

#include <commonroad_cpp/predicates/general/orientation_towards_predicate.h>

bool OrientationTowardsPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    auto ccsP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)->getCurvilinearCoordinateSystem()};
    return (obstacleK->getLatPosition(timeStep, ccsP) > // k on left side
                obstacleP->getLatPosition(world->getRoadNetwork(), timeStep) and
            obstacleK->getCurvilinearOrientation(timeStep, ccsP) < 0) or
           (obstacleK->getLatPosition(timeStep, ccsP) < // k on right side
                obstacleP->getLatPosition(world->getRoadNetwork(), timeStep) and
            obstacleK->getCurvilinearOrientation(timeStep, ccsP) > 0);
}

double OrientationTowardsPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("OrientationTowardsPredicate does not support robust evaluation!");
}

Constraint OrientationTowardsPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OrientationTowardsPredicate does not support constraint evaluation!");
}
OrientationTowardsPredicate::OrientationTowardsPredicate() : CommonRoadPredicate(true) {}
