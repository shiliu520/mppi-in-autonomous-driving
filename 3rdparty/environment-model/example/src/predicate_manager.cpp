#include "predicate_manager.h"
#include "command_line_input.h"
#include "yaml-cpp/yaml.h"
#include <boost/algorithm/string.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/obstacle/obstacle.h>
#include <commonroad_cpp/predicates/commonroad_predicate.h>
#include <commonroad_cpp/world.h>
#include <filesystem>
#include <fstream>
#include <omp.h>
#include <random>
#include <spdlog/spdlog.h>
#include <utility>

std::string PredicateManager::extractBenchmarkIdFromPath(std::string path) {
    std::vector<std::string> pathSplit;
    boost::split(pathSplit, path, boost::is_any_of("/"));
    std::string fileName{pathSplit.back()};
    std::vector<std::string> benchmarkNameSplit;
    boost::split(benchmarkNameSplit, fileName, boost::is_any_of("."));
    return benchmarkNameSplit.at(0);
}

void PredicateManager::extractPredicateSatisfaction() {
    spdlog::info("Start evaluation.");
    auto rng = std::default_random_engine{};
    // evaluate scenarios
    omp_set_num_threads(numThreads);
#pragma omp parallel for schedule(guided) shared(predicates, rng) firstprivate(scenarios, relevantPredicates)
    for (const auto &scen : scenarios) {
        std::string benchmarkId{extractBenchmarkIdFromPath(scen)};
        const auto &[obstacles, roadNetwork, timeStepSize, planningProblems] = InputUtils::getDataFromCommonRoad(scen);
        // evaluate all obstacles
        for (const auto &ego : obstacles) {
            if (ego->isStatic())
                continue;
            size_t minTimeStep{ego->getFirstTimeStep()};
            size_t maxTimeStep{ego->getFinalTimeStep()};
            std::vector<std::shared_ptr<Obstacle>> others;
            others.reserve(obstacles.size() - 1);
            for (const auto &obs : obstacles) {
                if (obs->getId() == ego->getId() or
                    (obs->getFinalTimeStep() < minTimeStep and maxTimeStep < obs->getFirstTimeStep()))
                    continue;
                others.push_back(obs);
            }
            // setObstacleProperties(ego, others); // todo
            const auto egoVehicles{std::vector<std::shared_ptr<Obstacle>>{ego}};
            const auto world{std::make_shared<World>(benchmarkId, 0, roadNetwork, egoVehicles, others, timeStepSize)};
            auto timer{std::make_shared<Timer>()};
            auto stat{std::make_shared<PredicateStatistics>()};
            for (size_t timeStep = ego->getCurrentState()->getTimeStep(); timeStep <= ego->getFinalTimeStep();
                 ++timeStep) {
                std::shuffle(std::begin(relevantPredicates), std::end(relevantPredicates), rng);
                for (const auto &predName : relevantPredicates) {
                    auto &pred{predicates[predName]};
                    //                    const std::shared_ptr<OptionalPredicateParameters> opt{
                    //                        std::make_shared<OptionalPredicateParameters>(
                    //                            std::vector<TrafficSignTypes>{TrafficSignTypes::MIN_SPEED},
                    //                            std::vector<LaneletType>{LaneletType::accessRamp},
                    //                            std::vector<TurningDirection>{TurningDirection::all},
                    //                            std::vector<TrafficLightState>{
                    //                                TrafficLightState::red})}; // TODO generalize for arbitrary types
                    try {
                        if (!pred->isVehicleDependent())
                            pred->statisticBooleanEvaluation(timeStep, world, ego, timer, stat, nullptr, {});
                        else
                            for (const auto &obs : world->getObstacles())
                                if (obs->timeStepExists(timeStep))
                                    pred->statisticBooleanEvaluation(timeStep, world, ego, timer, stat, obs, {});
                    } catch (const std::runtime_error &re) {
                        spdlog::error(std::string("PredicateManager::extractPredicateSatisfaction | Scenario: ")
                                          .append(scen)
                                          .append(" | ego vehicle: ")
                                          .append(std::to_string(ego->getId()))
                                          .append(" | predicate: ")
                                          .append(predName)
                                          .append(" | time step:")
                                          .append(std::to_string(timeStep))
                                          .append(" | Runtime Error: ")
                                          .append(re.what()));
                    } catch (const std::logic_error &le) {
                        spdlog::error(std::string("PredicateManager::extractPredicateSatisfaction | Scenario: ")
                                          .append(scen)
                                          .append(" | ego vehicle: ")
                                          .append(std::to_string(ego->getId()))
                                          .append(" | predicate: ")
                                          .append(predName)
                                          .append(" | time step:")
                                          .append(std::to_string(timeStep))
                                          .append(" | Logic Error: ")
                                          .append(le.what()));
                    } catch (const std::exception &ex) {
                        spdlog::error(std::string("PredicateManager::extractPredicateSatisfaction | Scenario: ")
                                          .append(scen)
                                          .append(" | ego vehicle: ")
                                          .append(std::to_string(ego->getId()))
                                          .append(" | predicate: ")
                                          .append(predName)
                                          .append(" | time step:")
                                          .append(std::to_string(timeStep))
                                          .append(" | General Error: ")
                                          .append(ex.what()));
                    } catch (...) {
                        spdlog::error(std::string("PredicateManager::extractPredicateSatisfaction | Scenario: ")
                                          .append(scen)
                                          .append(" | ego vehicle: ")
                                          .append(std::to_string(ego->getId()))
                                          .append(" | predicate: ")
                                          .append(predName)
                                          .append(" | time step:")
                                          .append(std::to_string(timeStep)));
                        continue;
                    }
                }
            }
        }
    }
    spdlog::info("Evaluation finished.");
    //   writeFile();
}

// void PredicateManager::writeFile() {
//    std::ofstream file;
//    double globalMinExecutionTime{std::numeric_limits<double>::max()};
//    double globalMaxExecutionTime{std::numeric_limits<double>::lowest()};
//    double globalMinAvgExecutionTime{std::numeric_limits<double>::max()};
//    double globalMaxAvgExecutionTime{std::numeric_limits<double>::lowest()};
//    if (std::filesystem::exists(simulationParameters.outputDirectory)) {
//        file.open(simulationParameters.outputDirectory + "/" + simulationParameters.outputFileName);
//        spdlog::info("Write evaluation results to " + simulationParameters.outputDirectory +
//                     simulationParameters.outputFileName);
//    } else {
//        file.open(std::filesystem::current_path() / simulationParameters.outputFileName);
//        spdlog::info(std::string("Write evaluation results to ")
//                         .append(std::filesystem::current_path())
//                         .append("/")
//                         .append(simulationParameters.outputFileName));
//    }
//    file << "Predicate Name,Num. Satisfactions,Num. Executions,Satisfaction in %,Max. Comp. Time,"
//            "Min. Comp. Time,Avg. Comp. Time,Weight Max. Comp. Time,Weight Avg. Comp. Time \n";
//    std::map<std::string, std::tuple<size_t, size_t, double, double, double, double>> predicateStatistics;
//    for (const auto &predName : relevantPredicates) {
//        auto& pred{predicates[predName]};
//        auto minExecutionTime{static_cast<double>(pred->getStatistics().minComputationTime) / 1e6};
//        auto maxExecutionTime{static_cast<double>(pred->getStatistics().maxComputationTime) / 1e6};
//        auto avgExecutionTime{(static_cast<double>(pred->getStatistics().totalComputationTime) / 1e6) /
//                              static_cast<double>(pred->getStatistics().numExecutions)};
//        if (minExecutionTime < globalMinExecutionTime)
//            globalMinExecutionTime = minExecutionTime;
//        if (maxExecutionTime > globalMaxExecutionTime)
//            globalMaxExecutionTime = maxExecutionTime;
//        if (avgExecutionTime < globalMinAvgExecutionTime)
//            globalMinAvgExecutionTime = avgExecutionTime;
//        if (avgExecutionTime > globalMaxAvgExecutionTime)
//            globalMaxAvgExecutionTime = avgExecutionTime;
//        predicateStatistics.insert({predName,
//                                    {pred->getStatistics().numSatisfaction, pred->getStatistics().numExecutions,
//                                     static_cast<double>(pred->getStatistics().numSatisfaction) /
//                                         static_cast<double>(pred->getStatistics().numExecutions),
//                                     maxExecutionTime, minExecutionTime, avgExecutionTime}});
//    }
//    for (const auto &[predName, pred] : predicateStatistics) {
//        file << predName << "," << std::get<0>(pred) << "," << std::get<1>(pred) << "," << std::get<2>(pred) << ","
//             << std::get<3>(pred) << " [ms]," << std::get<4>(pred) << " [ms]," << std::get<5>(pred) << " [ms],"
//             << (std::get<3>(pred) - globalMinExecutionTime) / (globalMaxExecutionTime - globalMinExecutionTime) <<
//             ","
//             << (std::get<5>(pred) - globalMinAvgExecutionTime) /
//                    (globalMaxAvgExecutionTime - globalMinAvgExecutionTime)
//             << "\n";
//    }
//    file << "Global Max. Comp. Time:," << std::to_string(globalMaxExecutionTime) << '\n';
//    file << "Global Min. Comp. Time:," << std::to_string(globalMinExecutionTime) << '\n';
//    file << "Global Max. Avg Comp. Time:," << std::to_string(globalMaxAvgExecutionTime) << '\n';
//    file << "Global Min. Avg Comp. Time:," << std::to_string(globalMinAvgExecutionTime) << '\n';
//    file.close();
//}

PredicateManager::PredicateManager(int threads, const std::string &configPath)
    : numThreads(threads), simulationParameters(InputUtils::initializeSimulationParameters(configPath)) {
    extractScenarios();
    extractRelevantPredicates(configPath);
}
void PredicateManager::extractScenarios() {
    EvaluationMode mode{simulationParameters.evaluationMode};
    std::string singleScenario{simulationParameters.benchmarkId};
    for (auto const &dir : simulationParameters.directoryPaths) {
        std::vector<std::string> fileNames{InputUtils::findRelevantScenarioFileNames(dir)};
        std::copy_if(fileNames.begin(), fileNames.end(), std::back_inserter(scenarios),
                     [mode, singleScenario](const std::string &name) {
                         return (mode == EvaluationMode::directory) or
                                ((mode == EvaluationMode::singleScenario or mode == EvaluationMode::singleVehicle) and
                                 name.find(singleScenario) != std::string::npos);
                     });
    }
}

PredicateManager::PredicateManager(int threads, SimulationParameters simulationParameters,
                                   std::vector<std::string> relevantPredicates)
    : numThreads(threads), simulationParameters(std::move(simulationParameters)),
      relevantPredicates(std::move(relevantPredicates)) {
    extractScenarios();
}

void PredicateManager::extractRelevantPredicates(const std::string &configPath) {
    YAML::Node config = YAML::LoadFile(configPath);
    auto relevantPredicateSets{config["predicate_eval"]["relevant_predicate_sets"].as<std::vector<std::string>>()};
    for (const auto &predicateSet : relevantPredicateSets) {
        auto relevantSetPredicates{
            config["predicate_eval"]["predicate_category"][predicateSet].as<std::vector<std::string>>()};
        for (const auto &pred : relevantSetPredicates)
            relevantPredicates.push_back(pred);
    }
}
void PredicateManager::reset() {
    //   for (const auto &predName : relevantPredicates)
    //       predicates[predName]->resetStatistics();
}
