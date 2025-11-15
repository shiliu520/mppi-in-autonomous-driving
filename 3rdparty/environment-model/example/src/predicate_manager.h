#pragma once

#include <commonroad_cpp/predicates/commonroad_predicate.h>
#include <commonroad_cpp/predicates/predicate_parameter_collection.h>
#include <memory>
#include <vector>

/**
 * Class for evaluating predicate satisfaction and computation time.
 */
class PredicateManager {
  public:
    /**
     * Constructor
     *
     * @param threads Number of threads which can be used.
     * @param configPath Path to configuration file which contains information about operation mode and traffic rules.
     */
    PredicateManager(int threads, const std::string &configPath);

    /**
     * Constructor
     *
     * @param threads Number of threads which can be used.
     * @param simulationParameters Struct containing simulation-related parameters.
     * @param relevantPredicates Subset of predicates which need to be evaluated.
     */
    PredicateManager(int threads, SimulationParameters simulationParameters,
                     std::vector<std::string> relevantPredicates);

    /**
     * Executes predicate evaluation.
     */
    void extractPredicateSatisfaction();

    /**
     * Resets predicate statistics.
     */
    void reset();

  private:
    std::vector<std::string> scenarios;          //**< set of scenario which should be evaluated */
    int numThreads;                              //**< number of threads which should be used for evaluation */
    SimulationParameters simulationParameters;   //**< struct containing simulation-related parameters */
    std::vector<std::string> relevantPredicates; //**< subset of relevant predicates which need to be evaluated */

    /**
     * Writes evaluation result to file.
     */
    void writeFile();

    /**
     * Collects names of relevant predicates.
     *
     * @param configPath Config file storing relevant predicates and predicate categories.
     */
    void extractRelevantPredicates(const std::string &configPath);

    /**
     * Collects complete path to scenarios within configured directories.
     */
    void extractScenarios();

    /**
     * Extract benchmark ID from path.
     * @param path File path.
     * @return Benchmark/Scenario ID.
     */
    std::string extractBenchmarkIdFromPath(std::string path);
};
