#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/general/makes_u_turn_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool MakesUTurnPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            bool setBased) {
    auto orientationCcs{std::fabs(obstacleK->getCurvilinearOrientation(world->getRoadNetwork(), timeStep))};
    return parameters.getParam("uTurnLower") <= orientationCcs and orientationCcs <= parameters.getParam("uTurnUpper");
}

Constraint MakesUTurnPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("Makes U Turn Predicate does not support constraint evaluation!");
}

double MakesUTurnPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             const std::vector<std::string> &additionalFunctionParameters,
                                             bool setBased) {
    throw std::runtime_error("Makes U Turn Predicate does not support robust evaluation!");
}

MakesUTurnPredicate::MakesUTurnPredicate() : CommonRoadPredicate(false) {}
