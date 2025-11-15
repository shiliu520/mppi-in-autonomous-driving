#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/misc/in_projection_domain_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/world.h>
#include <geometry/curvilinear_coordinate_system.h>

bool InProjectionDomainPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    auto referenceLaneP{obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)};

    return obstacleP->getReferenceLane(world->getRoadNetwork(), timeStep)
        ->getCurvilinearCoordinateSystem()
        ->cartesianPointInProjectionDomain(obstacleK->getStateByTimeStep(timeStep)->getXPosition(),
                                           obstacleK->getStateByTimeStep(timeStep)->getYPosition());
}

double InProjectionDomainPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("InProjectionDomainPredicate does not support robust evaluation!");
}

Constraint InProjectionDomainPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("InProjectionDomainPredicate does not support constraint evaluation!");
}
InProjectionDomainPredicate::InProjectionDomainPredicate() : CommonRoadPredicate(true) {}
