#include <commonroad_cpp/predicates/misc/high_computation_time_test_predicate.h>
#include <thread>

HighComputationTimeTestPredicate::HighComputationTimeTestPredicate() : CommonRoadPredicate(false) {}

bool HighComputationTimeTestPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

double HighComputationTimeTestPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                          const std::shared_ptr<Obstacle> &obstacleK,
                                                          const std::shared_ptr<Obstacle> &obstacleP,
                                                          const std::vector<std::string> &additionalFunctionParameters,
                                                          bool setBased) {
    return 0;
}

Constraint HighComputationTimeTestPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    return Constraint();
}
