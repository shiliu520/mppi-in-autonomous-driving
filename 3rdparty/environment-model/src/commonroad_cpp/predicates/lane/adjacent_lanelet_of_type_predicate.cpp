#include "commonroad_cpp/roadNetwork/lanelet/lanelet.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/lane/adjacent_lanelet_of_type_predicate.h>
#include <commonroad_cpp/world.h>

bool AdjacentLaneletOfTypePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                       const std::shared_ptr<Obstacle> &obstacleK,
                                                       const std::shared_ptr<Obstacle> &obstacleP,
                                                       const std::vector<std::string> &additionalFunctionParameters,
                                                       bool setBased) {
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        obstacleK->getOccupiedLaneletsByShape(world->getRoadNetwork(), timeStep);
    std::set<size_t> laneletIDs;
    for (const auto &la : lanelets)
        laneletIDs.insert(la->getId());
    std::vector<LaneletType> laTypes{lanelet_operations::matchStringToLaneletType(additionalFunctionParameters.at(1))};

    return std::any_of(
        lanelets.begin(), lanelets.end(),
        [laneletIDs, laTypes, additionalFunctionParameters](const std::shared_ptr<Lanelet> &lanelet) {
            std::vector<LaneletType> laTypesTmp;
            if (laTypes.size() == 1 and laTypes.at(0) == LaneletType::all)
                laTypesTmp.insert(laTypesTmp.end(), lanelet->getLaneletTypes().begin(),
                                  lanelet->getLaneletTypes().end());
            else
                laTypesTmp = laTypes;
            auto adjacent{
                lanelet->getAdjacent(regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)))
                    .adj};
            return adjacent != nullptr and adjacent->hasLaneletTypes(laTypesTmp) and
                   laneletIDs.find(adjacent->getId()) == laneletIDs.end();
        });
}

double AdjacentLaneletOfTypePredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters,
                                                        bool setBased) {
    throw std::runtime_error("AdjacentLaneletOfTypePredicate does not support robust evaluation!");
}

Constraint AdjacentLaneletOfTypePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("AdjacentLaneletOfTypePredicate does not support constraint evaluation!");
}

AdjacentLaneletOfTypePredicate::AdjacentLaneletOfTypePredicate() : CommonRoadPredicate(false) {}
