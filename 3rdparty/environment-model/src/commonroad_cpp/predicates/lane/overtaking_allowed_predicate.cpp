#include "commonroad_cpp/predicates/lane/overtaking_allowed_predicate.h"
#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include "commonroad_cpp/world.h"
#include <stdexcept>

OvertakingAllowedPredicate::OvertakingAllowedPredicate() : CommonRoadPredicate(false) {}

bool OvertakingAllowedPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                   const std::shared_ptr<Obstacle> &obstacleK,
                                                   const std::shared_ptr<Obstacle> &obstacleP,
                                                   const std::vector<std::string> &additionalFunctionParameters,
                                                   bool setBased) {

    // checking if line-marking or traffic sign of occupied lanelet prohibits overtaking
    auto occLanelets = obstacleK->getOccupiedLaneletsByBack(world->getRoadNetwork(), timeStep);
    if (std::any_of(occLanelets.begin(), occLanelets.end(), [](const std::shared_ptr<Lanelet> &lanelet) {
            return lanelet->getLineMarkingLeft() == LineMarking::solid ||
                   lanelet->getLineMarkingLeft() == LineMarking::solid_solid ||
                   lanelet->getLineMarkingLeft() == LineMarking::solid_dashed ||
                   lanelet->hasTrafficSign(TrafficSignTypes::NO_OVERTAKING_START) ||
                   lanelet->hasTrafficSign(TrafficSignTypes::NO_OVERTAKING_TRUCKS_START);
        }))
        return false;

    // check if already on no overtaking lane (next sign is a no-overtaking-end sign)
    auto lanes = obstacleK->getOccupiedLanes(world->getRoadNetwork(), timeStep);
    for (const auto &lane : lanes) {
        // lane can contain already used lanelets on a route and this should only check successors of the occupied
        // lanelets
        bool predecessorsOfOccupied{true};
        for (const auto &lanelet : lane->getContainedLanelets()) {
            // check if lanelet already visited and skip if true
            if (predecessorsOfOccupied) {
                if (std::find(occLanelets.begin(), occLanelets.end(), lanelet) != occLanelets.end())
                    predecessorsOfOccupied = false;
                continue;
            }
            if (lanelet->hasTrafficSign(TrafficSignTypes::NO_OVERTAKING_END) ||
                lanelet->hasTrafficSign(TrafficSignTypes::NO_OVERTAKING_TRUCKS_END))
                return false;
            if (lanelet->hasTrafficSign(TrafficSignTypes::NO_OVERTAKING_START) ||
                lanelet->hasTrafficSign(TrafficSignTypes::NO_OVERTAKING_TRUCKS_START)) {
                break;
            }
        }
    }
    return true;
}

double OvertakingAllowedPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                    const std::shared_ptr<Obstacle> &obstacleK,
                                                    const std::shared_ptr<Obstacle> &obstacleP,
                                                    const std::vector<std::string> &additionalFunctionParameters,
                                                    bool setBased) {
    throw std::runtime_error("OvertakingAllowedPredicate does not support robust evaluation!");
}

Constraint OvertakingAllowedPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    throw std::runtime_error("OvertakingAllowedPredicate does not support constraint evaluation!");
}
