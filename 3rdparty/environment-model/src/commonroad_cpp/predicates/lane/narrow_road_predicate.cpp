#include "commonroad_cpp/predicates/lane/narrow_road_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/world.h"

bool NarrowRoadPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Obstacle> &obstacleK,
                                            const std::shared_ptr<Obstacle> &obstacleP,
                                            const std::vector<std::string> &additionalFunctionParameters,
                                            bool setBased) {
    auto occLaneletsFront{obstacleK->getOccupiedLaneletsByFront(world->getRoadNetwork(), timeStep)};
    auto occLaneletsRear{obstacleK->getOccupiedLaneletsByFront(world->getRoadNetwork(), timeStep)};
    if (std::all_of(occLaneletsFront.begin(), occLaneletsFront.end(),
                    [this](const std::shared_ptr<Lanelet> &lanelet) {
                        return lanelet->getMinWidth() > parameters.getParam("narrowRoad");
                    }) and
        std::all_of(occLaneletsRear.begin(), occLaneletsRear.end(), [this](const std::shared_ptr<Lanelet> &lanelet) {
            return lanelet->getMinWidth() > parameters.getParam("narrowRoad");
        }))
        return false;
    if (std::any_of(occLaneletsFront.begin(), occLaneletsFront.end(),
                    [this, obstacleK, timeStep](const std::shared_ptr<Lanelet> &lanelet) {
                        auto obsPos{obstacleK->getFrontXYCoordinates(timeStep)};
                        return lanelet_operations::roadWidth(lanelet, obsPos.at(0), obsPos.at(1)) <=
                               parameters.getParam("narrowRoad");
                    }) or
        std::any_of(occLaneletsRear.begin(), occLaneletsRear.end(),
                    [this, obstacleK, timeStep](const std::shared_ptr<Lanelet> &lanelet) {
                        auto obsPos{obstacleK->getBackXYCoordinates(timeStep)};
                        return lanelet_operations::roadWidth(lanelet, obsPos.at(0), obsPos.at(1)) <=
                               parameters.getParam("narrowRoad");
                    }))
        return true;
    return false;
}

Constraint NarrowRoadPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                     const std::shared_ptr<Obstacle> &obstacleK,
                                                     const std::shared_ptr<Obstacle> &obstacleP,
                                                     const std::vector<std::string> &additionalFunctionParameters,
                                                     bool setBased) {
    throw std::runtime_error("Narrow Road Predicate does not support constraint evaluation!");
}

double NarrowRoadPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             const std::vector<std::string> &additionalFunctionParameters,
                                             bool setBased) {
    throw std::runtime_error("Narrow Road Predicate does not support robust evaluation!");
}

NarrowRoadPredicate::NarrowRoadPredicate() : CommonRoadPredicate(false) {}
