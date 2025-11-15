#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h"
#include "commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h"
#include "commonroad_cpp/world.h"
#include <commonroad_cpp/obstacle/obstacle_operations.h>
#include <commonroad_cpp/predicates/lane/parallel_to_lane_marking_of_type_on_side_predicate.h>

bool ParallelToLaneMarkingOfTypeOnSidePredicate::booleanEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    // get relevant line marking types
    std::vector<LineMarking> lineMarkingTypes{
        lanelet_operations::matchStringToLineMarkingOptions(additionalFunctionParameters.at(1))};
    if (lanelet_operations::anyLaneletsContainLineMarkingType(
            obstacleK->getOccupiedLaneletsDrivingDirectionByShape(world->getRoadNetwork(), timeStep), lineMarkingTypes,
            additionalFunctionParameters.at(0)))
        return false;

    std::vector<std::shared_ptr<Lanelet>> lanelets{};
    std::set<std::shared_ptr<Lanelet>> adjLanelets;
    if (regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)) == Direction::left) {
        adjLanelets = obstacle_operations::laneletsRightOfObstacle(timeStep, world->getRoadNetwork(), obstacleK);
    } else if (regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)) == Direction::right) {
        adjLanelets = obstacle_operations::laneletsLeftOfObstacle(timeStep, world->getRoadNetwork(), obstacleK);
    } else
        throw std::runtime_error(
            "ParallelToLaneMarkingOfTypeOnSidePredicate::booleanEvaluation: Invalid turning direction.");
    lanelets.insert(lanelets.end(), adjLanelets.begin(), adjLanelets.end());
    return lanelet_operations::anyLaneletsContainLineMarkingType(lanelets, lineMarkingTypes,
                                                                 additionalFunctionParameters.at(0));
}

double ParallelToLaneMarkingOfTypeOnSidePredicate::robustEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("ParallelToLaneMarkingOfTypeOnSidePredicate does not support robust evaluation!");
}

Constraint ParallelToLaneMarkingOfTypeOnSidePredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("ParallelToLaneMarkingOfTypeOnSidePredicate does not support constraint evaluation!");
}

ParallelToLaneMarkingOfTypeOnSidePredicate::ParallelToLaneMarkingOfTypeOnSidePredicate() : CommonRoadPredicate(false) {}
