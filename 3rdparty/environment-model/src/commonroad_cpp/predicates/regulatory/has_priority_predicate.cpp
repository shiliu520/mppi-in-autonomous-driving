#include <commonroad_cpp/predicates/regulatory/has_priority_predicate.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h>
#include <commonroad_cpp/world.h>
#include <memory>

bool HasPriorityPredicate::booleanEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                             const std::shared_ptr<Obstacle> &obstacleK,
                                             const std::shared_ptr<Obstacle> &obstacleP,
                                             const std::vector<std::string> &additionalFunctionParameters,
                                             bool setBased) {
    int prioK{regulatory_elements_utils::getPriority(
        timeStep, world->getRoadNetwork(), obstacleK,
        regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(0)))};
    int prioP{regulatory_elements_utils::getPriority(
        timeStep, world->getRoadNetwork(), obstacleP,
        regulatory_elements_utils::matchDirections(additionalFunctionParameters.at(1)))};
    return prioK > prioP and prioK != std::numeric_limits<int>::min() and prioP != std::numeric_limits<int>::min();
}

double HasPriorityPredicate::robustEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                              const std::shared_ptr<Obstacle> &obstacleK,
                                              const std::shared_ptr<Obstacle> &obstacleP,
                                              const std::vector<std::string> &additionalFunctionParameters,
                                              bool setBased) {
    throw std::runtime_error("HasPriorityPredicate does not support robust evaluation!");
}

Constraint HasPriorityPredicate::constraintEvaluation(size_t timeStep, const std::shared_ptr<World> &world,
                                                      const std::shared_ptr<Obstacle> &obstacleK,
                                                      const std::shared_ptr<Obstacle> &obstacleP,
                                                      const std::vector<std::string> &additionalFunctionParameters,
                                                      bool setBased) {
    throw std::runtime_error("HasPriorityPredicate does not support constraint evaluation!");
}

HasPriorityPredicate::HasPriorityPredicate() : CommonRoadPredicate(true) {}
