#include <boost/algorithm/string.hpp>
#include <commonroad_cpp/interfaces/commonroad/input_utils.h>
#include <commonroad_cpp/interfaces/commonroad/protobuf_reader.h>
#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <commonroad_cpp/planning_problem.h>
#include <commonroad_cpp/roadNetwork/intersection/intersection.h>
#include <commonroad_cpp/roadNetwork/lanelet/lanelet.h>
#include <commonroad_cpp/roadNetwork/road_network.h>
#include <filesystem>
#include <spdlog/spdlog.h>

namespace {

/**
 * Reads CR scenario from file in xml format.
 *
 * @param xmlFilePath File path
 * @return Scenario
 */
Scenario readFromXMLFile(const std::string &xmlFilePath) {
    std::vector<std::shared_ptr<TrafficSign>> trafficSigns = XMLReader::createTrafficSignFromXML(xmlFilePath);
    std::vector<std::shared_ptr<TrafficLight>> trafficLights = XMLReader::createTrafficLightFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Lanelet>> lanelets =
        XMLReader::createLaneletFromXML(xmlFilePath, trafficSigns, trafficLights);
    std::vector<std::shared_ptr<Obstacle>> obstacles = XMLReader::createObstacleFromXML(xmlFilePath);
    std::vector<std::shared_ptr<Intersection>> intersections =
        XMLReader::createIntersectionFromXML(xmlFilePath, lanelets);
    auto country{XMLReader::extractCountryFromXML(xmlFilePath)};

    std::shared_ptr<RoadNetwork> roadNetwork{
        std::make_shared<RoadNetwork>(RoadNetwork(lanelets, country, trafficSigns, trafficLights, intersections))};
    for (const auto &inter : roadNetwork->getIntersections())
        inter->computeMemberLanelets(roadNetwork);

    auto timeStepSize{XMLReader::extractTimeStepSize(xmlFilePath)};

    std::vector<std::shared_ptr<PlanningProblem>> planningProblems{
        XMLReader::createPlanningProblemFromXML(xmlFilePath)};

    return Scenario{obstacles, roadNetwork, timeStepSize, planningProblems};
}

/**
 * Reads CR scenario from file in protobuf format.
 *
 * @param pbFilePath File path
 * @return Scenario
 */
Scenario readFromProtobufFile(const std::string &pbFilePath) {
    commonroad_dynamic::CommonRoadDynamic commonRoadDynamicMsg;
    commonroad_map::CommonRoadMap commonRoadMapMsg;
    commonroad_scenario::CommonRoadScenario commonRoadScenarioMsg;
    std::vector<std::string> dirSplit;
    boost::split(dirSplit, pbFilePath, boost::is_any_of("/"));
    std::string dir;
    int fileGiven{0};
    for (size_t idx{0}; idx < dirSplit.size() - 1; ++idx)
        dir += dirSplit[idx] + "/";
    std::string name;
    if (dirSplit.back().find("SC") != std::string::npos)
        name = dirSplit.back().substr(0, dirSplit.back().size() - 6);
    else
        name = dirSplit.back().substr(0, dirSplit.back().size() - 3);
    for (const auto &entry : std::filesystem::directory_iterator(dir)) {
        std::vector<std::string> pathSplit;
        boost::split(pathSplit, entry.path().string(), boost::is_any_of("/"));
        if (std::count(pathSplit.back().begin(), pathSplit.back().end(), '_') == 1) {
            fileGiven = fileGiven | 1;
            commonRoadMapMsg = ProtobufReader::loadMapProtobufMessage(entry.path().string());
        } else if (pathSplit.back().find("SC") != std::string::npos and
                   pathSplit.back().find(name) != std::string::npos) {
            fileGiven = fileGiven | 4;
            commonRoadScenarioMsg = ProtobufReader::loadScenarioProtobufMessage(entry.path().string());
        } else if (pathSplit.back().find(name) != std::string::npos) {
            commonRoadDynamicMsg = ProtobufReader::loadDynamicProtobufMessage(entry.path().string());
            fileGiven = fileGiven | 2;
        }
    }

    return ProtobufReader::createCommonRoadFromMessage(commonRoadDynamicMsg, commonRoadMapMsg, commonRoadScenarioMsg,
                                                       fileGiven);
}

} // namespace

Scenario InputUtils::getDataFromCommonRoad(const std::string &path) {
    spdlog::debug("Read file: {}", path);
    std::vector<std::string> fileEndingSplit;
    boost::split(fileEndingSplit, path, boost::is_any_of("."));
    Scenario scenario;
    if (fileEndingSplit.back() == "xml")
        scenario = readFromXMLFile(path);
    else if (fileEndingSplit.back() == "pb")
        scenario = readFromProtobufFile(path);
    else
        throw std::runtime_error("Invalid file name " + path + ": .xml or .pb ending missing!");

    spdlog::debug("File successfully read: {}", path);
    return scenario;
}
