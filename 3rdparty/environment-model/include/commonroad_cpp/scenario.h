#pragma once

#include <memory>
#include <vector>

#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/planning_problem.h>

// using Scenario = std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double>;

struct Scenario {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
    std::shared_ptr<RoadNetwork> roadNetwork;
    double timeStepSize;
    std::vector<std::shared_ptr<PlanningProblem>> planningProblems;
};
