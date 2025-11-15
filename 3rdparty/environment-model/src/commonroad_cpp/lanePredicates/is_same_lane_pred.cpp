#include "commonroad_cpp/roadNetwork/lanelet/lane.h"
#include <commonroad_cpp/lanePredicates/is_same_lane_pred.h>
#include <stdexcept>

bool IsSameLanePredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                            const std::shared_ptr<Lane> &laneK, const std::shared_ptr<Lane> &laneP,
                                            const std::vector<std::string> &additionalFunctionParameters) {
    if (laneK == nullptr or laneP == nullptr)
        throw std::invalid_argument("IsSameLanePredicate::booleanEvaluation: One of the lanes is not valid.");
    return laneK->getId() == laneP->getId();
}
