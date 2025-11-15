#pragma once

#include "string"
#include "types_and_definitions.h"
#include "vector"
#include <climits>
#include <cmath>
#include <cstddef>
#include <utility>

struct TrafficLightCycleElement {
    TrafficLightState color;
    size_t duration;
};

struct vertex {
    double x{0};
    double y{0};
    vertex() = default;
    vertex(std::initializer_list<double> list) {
        const auto *iter = list.begin();
        switch (list.size()) {
        default:
        case 2:
            x = *iter++;
            y = *iter++;
            break;
        case 1:
            x = y = *iter;
            break;
        case 0:
            break;
        };
    }
    vertex operator+(const vertex &vert) const { return {this->x + vert.x, this->y + vert.y}; }
    vertex operator-(const vertex &vert) const { return {this->x - vert.x, this->y - vert.y}; }
    vertex operator*(const vertex &vert) const { return {this->x * vert.x, this->y * vert.y}; }
    vertex operator*(const double &scalar) const { return {this->x * scalar, this->y * scalar}; }
    vertex operator/(const vertex &vert) const { return {this->x / vert.x, this->y / vert.y}; }
    vertex operator/(const double &scalar) const { return {this->x / scalar, this->y / scalar}; }
    void operator+=(const vertex &vert) {
        this->x += vert.x;
        this->y += vert.y;
    }
    void operator-=(const vertex &vert) {
        this->x -= vert.x;
        this->y -= vert.y;
    }
    bool operator==(const vertex &vert) const {
        float epsilon = std::numeric_limits<double>::epsilon();
        return fabs(this->x - vert.x) < epsilon and fabs(this->y - vert.y) < epsilon;
    }
};

struct ValidStates {
    bool xPosition;
    bool yPosition;
    bool velocity;
    bool acceleration;
    bool lonPosition;
    bool latPosition;
    bool globalOrientation;
    bool curvilinearOrientation;
};

struct PredicateStatistics {
    std::vector<double> computationTime; // ms
    size_t totalComputationTime{0};      // ns
    size_t numExecutions{0};
    size_t numSatisfaction{0};

    void reset() {
        computationTime.clear();
        totalComputationTime = 0;
        numExecutions = 0;
        numSatisfaction = 0;
    }
};

struct Constraint {
    double realValuedConstraint;
};

struct SimulationParameters {
    SimulationParameters(std::vector<std::string> directoryPaths, size_t egoVehicleId, std::string benchmarkId,
                         EvaluationMode evaluationMode, bool performanceMeasurement, std::string outputDirectory,
                         std::string outputFileName, bool checkObstacleValid, bool removeVRU, bool checkEgoValid)
        : directoryPaths(std::move(directoryPaths)), egoVehicleId(egoVehicleId), benchmarkId(std::move(benchmarkId)),
          evaluationMode(evaluationMode), performanceMeasurement(performanceMeasurement),
          checkObstacleValid(checkObstacleValid), removeVRU(removeVRU), checkEgoValid(checkEgoValid),
          outputDirectory(std::move(outputDirectory)), outputFileName(std::move(outputFileName)) {};
    SimulationParameters() = default;

    std::vector<std::string> directoryPaths{}; //**< List of directories in which all scenarios should be evaluated */
    size_t egoVehicleId{0};                    //**< ID of ego vehicle */
    std::string benchmarkId{"DEU_test_max_speed_limit"}; //**< CommonRoad benchmark ID */
    EvaluationMode evaluationMode{
        EvaluationMode::directory}; //**< Evaluation mode which should be used, e.g., directory, single vehicle, ... */
    bool performanceMeasurement{true}; //**< Flag indicating whether performance should be measured. */
    bool checkObstacleValid{
        true}; //**< Boolean indicating whether obstacles out of road network or always standing should be ignored. */
    bool removeVRU{true};     //**< Boolean indicating whether VRU obstacles should be ignored. */
    bool checkEgoValid{true}; //**< Boolean indicating whether ego vehicles out of road network or always standing
                              // should be ignored. */
    std::string outputDirectory{"path/to/output/directory"};   //**< Path to output directory of file to generate. */
    std::string outputFileName{"traffic_rule_evaluation.txt"}; //**< name and file type for to generate. */
};
