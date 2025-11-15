#include <string>
#include <tuple>
#include <vector>

#include <commonroad_cpp/interfaces/commonroad/xml_reader.h>
#include <commonroad_cpp/planning_problem.h>
#include <commonroad_cpp/scenario.h>

class RoadNetwork;

// using Scenario = std::tuple<std::vector<std::shared_ptr<Obstacle>>, std::shared_ptr<RoadNetwork>, double>;

namespace InputUtils {

/**
 * Loads and sets up CR scenario. The file/directory can be encoded in xml or protobuf format.
 *
 * @param path Path to CommonRoad file/directory
 * @return Tuple of obstacles, roadNetwork, and time step size
 */
Scenario getDataFromCommonRoad(const std::string &path);

PlanningProblem getPlanningProblemFromCommonRoad(const std::string &path);

} // namespace InputUtils
