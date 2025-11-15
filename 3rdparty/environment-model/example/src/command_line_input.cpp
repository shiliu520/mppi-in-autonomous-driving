#include "commonroad_cpp/obstacle/obstacle.h"
#include "commonroad_cpp/roadNetwork/road_network.h"

#include <boost/algorithm/string/predicate.hpp>

#include <boost/program_options.hpp>
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <iostream>

#include "command_line_input.h"
#include "yaml-cpp/yaml.h"

#include <filesystem>
#include <spdlog/spdlog.h>

namespace po = boost::program_options;

/**
 * Reads the arguments provided via command line.
 *
 * @param argc Argument count: Number of arguments.
 * @param argv Argument vector: List of arguments. (Argument at position 0 is the executable)
 * @param num_threads Reference where number of threads should be stored.
 * @param xmlFilePath Reference where file path to CommonRoad xml should be stored.
 */
int InputUtils::readCommandLineValues(int argc, char *const *argv, int &num_threads, std::string &xmlFilePath) {
    try {
        std::string xmlFileName;
        po::options_description desc;
        po::variables_map variablesMap;
        desc.add_options()("help", "produce help message")(
            "input-file",
            boost::program_options::value<std::string>(&xmlFilePath)
                ->default_value("../src/commonroad_cpp_tests/default_config.xml")
                ->required(),
            "Input file")("threads,t", po::value<int>(&num_threads)->default_value(1),
                          "set number of threads to run with");
        po::positional_options_description positionalOptionsDescription;
        positionalOptionsDescription.add("input-file", -1);
        boost::program_options::store(
            po::command_line_parser(argc, argv).options(desc).positional(positionalOptionsDescription).run(),
            variablesMap);
        po::notify(variablesMap);

        if (variablesMap.count("help") != 0u) {
            std::cout << desc << "\n";
            return 0;
        }

        spdlog::info("Using file {}", xmlFilePath);
        spdlog::info("Thread count: {}", num_threads);

        return 0;
    } catch (std::exception &e) {
        spdlog::error(std::string{"InputUtils::readCommandLineValues: "} + e.what());
        return 1;
    } catch (...) {
        std::cerr << "InputUtils::readCommandLineValues: Exception of unknown type!\n";
        return 1;
    }
}

SimulationParameters InputUtils::initializeSimulationParameters(const std::string &configPath) {
    YAML::Node config = YAML::LoadFile(configPath);
    return {config["simulation_param"]["directories"].as<std::vector<std::string>>(),
            config["simulation_param"]["ego_vehicle_id"].as<size_t>(),
            config["simulation_param"]["benchmark_id"].as<std::string>(),
            stringToEvaluationMode(config["simulation_param"]["evaluation_mode"].as<std::string>()),
            config["simulation_param"]["performance_measurement"].as<bool>(),
            config["simulation_param"]["output_directory"].as<std::string>(),
            config["simulation_param"]["output_file_name"].as<std::string>(),
            config["simulation_param"]["checkObstacleValid"].as<bool>(),
            config["simulation_param"]["removeVRU"].as<bool>(),
            config["simulation_param"]["checkEgoValid"].as<bool>()};
}

EvaluationMode InputUtils::stringToEvaluationMode(const std::string &evalMode) {
    if (evalMode == "directory") {
        return EvaluationMode::directory;
    }
    if (evalMode == "single_scenario") {
        return EvaluationMode::singleScenario;
    }
    if (evalMode == "single_vehicle") {
        return EvaluationMode::singleVehicle;
    }
    if (evalMode == "directory_single_vehicle") {
        return EvaluationMode::directory_single_vehicle;
    }
    throw std::runtime_error("CommonRoadEvaluation: Unknown evaluation mode.\n Options are: directory, "
                             "single_scenario, single_vehicle, directory_single_vehicle");
}

std::vector<std::string> InputUtils::findRelevantScenarioFileNames(const std::string &dir) {
    std::vector<std::string> fileNames;
    for (std::filesystem::directory_iterator itr(dir); itr != std::filesystem::directory_iterator(); ++itr)
        if (boost::algorithm::ends_with(itr->path().string(), ".xml"))
            fileNames.push_back(itr->path().string());
    return fileNames;
}
