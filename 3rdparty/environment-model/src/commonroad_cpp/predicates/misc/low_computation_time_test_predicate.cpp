#include <commonroad_cpp/predicates/misc/low_computation_time_test_predicate.h>
#include <thread>

LowComputationTimeTestPredicate::LowComputationTimeTestPredicate() : CommonRoadPredicate(false) {}

bool LowComputationTimeTestPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                        const std::shared_ptr<Obstacle> &obstacleK,
                                                        const std::shared_ptr<Obstacle> &obstacleP,
                                                        const std::vector<std::string> &additionalFunctionParameters,
                                                        bool setBased) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return true;
}

double LowComputationTimeTestPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                         const std::shared_ptr<Obstacle> &obstacleK,
                                                         const std::shared_ptr<Obstacle> &obstacleP,
                                                         const std::vector<std::string> &additionalFunctionParameters,
                                                         bool setBased) {
    return 0;
}

Constraint LowComputationTimeTestPredicate::constraintEvaluation(
    size_t timeStep, const std::shared_ptr<World> &world, const std::shared_ptr<Obstacle> &obstacleK,
    const std::shared_ptr<Obstacle> &obstacleP, const std::vector<std::string> &additionalFunctionParameters,
    bool setBased) {
    return Constraint();
}
