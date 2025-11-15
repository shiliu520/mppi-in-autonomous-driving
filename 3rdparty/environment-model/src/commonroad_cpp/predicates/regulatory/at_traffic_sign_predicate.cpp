#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/road_network.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/regulatory/at_traffic_sign_predicate.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h>
#include <commonroad_cpp/world.h>

bool AtTrafficSignPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                               const std::shared_ptr<Obstacle> &obstacleK,
                                               const std::shared_ptr<Obstacle> &obstacleP,
                                               const std::vector<std::string> &additionalFunctionParameters,
                                               bool setBased) {

    const auto signId{TrafficSign::matchTrafficSign(additionalFunctionParameters.at(0))};
    for (const auto &lanelet : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep))
        for (const auto &sign : lanelet->getTrafficSigns())
            if (!sign->getTrafficSignElementsOfType(signId).empty())
                return true;

    return false;
}

double AtTrafficSignPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                const std::shared_ptr<Obstacle> &obstacleK,
                                                const std::shared_ptr<Obstacle> &obstacleP,
                                                const std::vector<std::string> &additionalFunctionParameters,
                                                bool setBased) {
    throw std::runtime_error("AtTrafficSignPredicate does not support robust evaluation!");
}
Constraint AtTrafficSignPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters,
                                                        bool setBased) {
    throw std::runtime_error("AtTrafficSignPredicate does not support constraint evaluation!");
}
AtTrafficSignPredicate::AtTrafficSignPredicate() : CommonRoadPredicate(false) {}
