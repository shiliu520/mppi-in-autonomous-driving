#include "commonroad_cpp/lanePredicates/is_same_lane_pred.h"
#include <commonroad_cpp/lanePredicates/commonroad_lane.h>

bool CommonRoadLanePredicate::statisticBooleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Lane> &laneK,
                                                         const std::shared_ptr<Timer> &evaluationTimer,
                                                         const std::shared_ptr<PredicateStatistics> &statistics,
                                                         const std::shared_ptr<Lane> &laneP,
                                                         const std::vector<std::string> &additionalFunctionParameters) {
    auto startTime{Timer::start()};
    bool result{booleanEvaluation(timeStep, world, laneK, laneP, additionalFunctionParameters)};
    long compTime{evaluationTimer->stop(startTime)};

    statistics->numExecutions++;
    statistics->totalComputationTime += static_cast<unsigned long>(compTime);
    statistics->computationTime.push_back(static_cast<double>(compTime) / 1e6);
    if (result)
        statistics->numSatisfaction++;

    return result;
}

PredicateParameters &CommonRoadLanePredicate::getParameters() { return parameters; }

std::map<std::string, std::shared_ptr<CommonRoadLanePredicate>> lanePredicates{
    {"is_same_lane_predicate", std::make_shared<IsSameLanePredicate>()}};
