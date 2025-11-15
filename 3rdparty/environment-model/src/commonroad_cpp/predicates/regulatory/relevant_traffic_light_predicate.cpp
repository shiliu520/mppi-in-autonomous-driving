#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/regulatory/relevant_traffic_light_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lane.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <commonroad_cpp/world.h>

bool RelevantTrafficLightPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    if (!regulatory_elements_utils::activeTrafficLights(timeStep, obstacleK, world->getRoadNetwork()).empty())
        return true;
    else {
        for (const auto &lanelet : obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep)) {
            auto lanes{world->getRoadNetwork()->findLanesByBaseLanelet(lanelet->getId())};
            for (const auto &lane : lanes) {
                auto relevantLanelets{lane->getSuccessorLanelets(lanelet)};
                for (const auto &let : relevantLanelets) {
                    for (const auto &light : let->getTrafficLights())
                        if (light->isActive() or light->getElementAtTime(timeStep).color != TrafficLightState::inactive)
                            return true;
                }
            }
        }
    }
    return false;
}

double RelevantTrafficLightPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    throw std::runtime_error("RelevantTrafficLightPredicate does not support robust evaluation!");
}
Constraint RelevantTrafficLightPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("RelevantTrafficLightPredicate does not support constraint evaluation!");
}
RelevantTrafficLightPredicate::RelevantTrafficLightPredicate() : CommonRoadPredicate(false) {}
